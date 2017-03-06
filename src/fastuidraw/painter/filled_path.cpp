/*!
 * \file filled_path.cpp
 * \brief file filled_path.cpp
 *
 * Copyright 2016 by Intel.
 *
 * Contact: kevin.rogovin@intel.com
 *
 * This Source Code Form is subject to the
 * terms of the Mozilla Public License, v. 2.0.
 * If a copy of the MPL was not distributed with
 * this file, You can obtain one at
 * http://mozilla.org/MPL/2.0/.
 *
 * \author Kevin Rogovin <kevin.rogovin@intel.com>
 *
 */


#include <vector>
#include <list>
#include <map>
#include <set>
#include <algorithm>
#include <ctime>
#include <math.h>

#include <fastuidraw/tessellated_path.hpp>
#include <fastuidraw/path.hpp>
#include <fastuidraw/painter/filled_path.hpp>
#include <fastuidraw/painter/painter_attribute_data.hpp>
#include "../private/util_private.hpp"
#include "../private/util_private_ostream.hpp"
#include "../private/bounding_box.hpp"
#include "../private/clip.hpp"
#include "../../3rd_party/glu-tess/glu-tess.hpp"

/* Actual triangulation is handled by GLU-tess.
   The main complexity in creating a FilledPath
   comes from two elements:
    - handling overlapping edges
    - creating a hierarchy for creating triangulations
      and for culling.

   The first is needed because GLU-tess will fail
   if any two edges overlap (we say a pair of edges
   overlap if they intersect at more than just a single
   point). We handle this by observing that GLU-tess
   takes doubles but TessellatedPath is floats. When
   we feed the coordinates to GLU-tess, we offset the
   values by an amount that is visible in fp64 but not
   in fp32. In addition, we also want to merge points
   that are close in fp32 as well. The details are
   handled in CoordinateCoverter, PointHoard and
   tesser.

   The second is needed for primarily to speed up
   tessellation. If a TessellatedPath has a large
   number of vertices, then that is likely because
   it is a high level of detail and likely zoomed in
   a great deal. To handle that, we need only to
   have the triangulation of a smaller portion of
   it ready. Thus we break the original path into
   a hierarchy of paths. The partitioning is done
   a single half plane at a time. A contour from
   the original path is computed by simply removing
   any points on the wrong side of the half plane
   and inserting the points where the path crossed
   the half plane. The sub-path objects are computed
   via the class SubPath. The class SubsetPrivate
   is the one that represents an element in the
   hierarchy that is triangulated on demand.
 */

/* Values to define how to create Subset objects.
 */
namespace SubsetConstants
{
  enum
    {
      recursion_depth = 12,
      points_per_subset = 64
    };

  /* if negative, aspect ratio is not
     enfored.
   */
  const float size_max_ratio = 4.0f;
}

/* Constants for CoordinateConverter.
   CoordinateConverter's purpose is to remap
   the bounding box of a fastuidraw::TessellatedPath
   to [0, 2 ^ N] x [0,  2 ^ N]
   and then apply a fudge offset to the point
   that an fp64 sees but an fp32 does not.

   We do this to allow for the input TessellatedPath
   to have overlapping edges. The value for the
   fudge offset is to be incremented on each point.

   An fp32 has a 23-bit significand that allows it
   to represent any integer in the range [-2^24, 2^24]
   exactly. An fp64 has a 52 bit significand.

   We set N to be 24 and the fudginess to be 2^-20
   (leaving 9-bits for GLU to use for intersections).
 */
namespace CoordinateConverterConstants
{
  enum
    {
      log2_box_dim = 16,
      negative_log2_fudge = 20,
      box_dim = (1 << log2_box_dim),
    };
}

namespace
{

  class winding_set
  {
  public:
    winding_set(void):
      m_min_value(1),
      m_max_value(0)
    {}

    bool
    operator()(int V) const
    {
      return m_values[compute_index(V)] != 0;
    }

    void
    clear(void)
    {
      m_values.clear();
    }

    void
    fill(int min_value, int max_value,
         const fastuidraw::CustomFillRuleBase &fill_rule)
    {
      m_values.clear();
      m_min_value = min_value;
      m_max_value = max_value;
      m_values.resize(fastuidraw::t_max(1 + m_max_value - m_min_value, 0), 0);
      for(int w = min_value; w <= max_value; ++w)
        {
          m_values[compute_index(w)] = fill_rule(w) ? 1u : 0u;
        }
    }

  private:

    unsigned int
    compute_index(int v) const
    {
      #ifdef FASTUIDRAW_VECTOR_BOUND_CHECK
      assert(v >= m_min_value && v <= m_max_value);
      #endif
      return v - m_min_value;
    }

    /* NOTE:
        we use an array of uint32_t's that although takes
        more storage, has faster element access (because
        to access an element does not require any bit-logic)
        and a winding_set is used as a cache for output to
        a custom fill rule.
     */
    int m_min_value, m_max_value;
    std::vector<uint32_t> m_values;
  };

  class per_winding_data:
    public fastuidraw::reference_counted<per_winding_data>::non_concurrent
  {
  public:
    typedef std::list<unsigned int> index_list;

    per_winding_data(void):
      m_count(0)
    {}

    void
    add_index(unsigned int idx)
    {
      m_indices.push_back(idx);
      ++m_count;
    }

    const index_list&
    indices(void) const
    {
      return m_indices;
    }

    unsigned int
    count(void) const
    {
      return m_count;
    }

    void
    fill_at(unsigned int &offset,
            fastuidraw::c_array<unsigned int> dest,
            fastuidraw::const_c_array<unsigned int> &sub_range)
    {
      assert(count() + offset <= dest.size());
      std::copy(m_indices.begin(), m_indices.end(), &dest[offset]);
      sub_range = dest.sub_array(offset, count());
      offset += count();
    }

  private:
    index_list m_indices;
    unsigned int m_count;
  };

  typedef std::map<int, fastuidraw::reference_counted_ptr<per_winding_data> > winding_index_hoard;

  bool
  is_even(int v)
  {
    return (v % 2) == 0;
  }

  class CoordinateConverter
  {
  public:
    explicit
    CoordinateConverter(const fastuidraw::vec2 &fpmin, const fastuidraw::vec2 &fpmax)
    {
      fastuidraw::vecN<double, 2> delta, pmin, pmax;

      pmin = fastuidraw::vecN<double, 2>(fpmin);
      pmax = fastuidraw::vecN<double, 2>(fpmax);
      delta = pmax - pmin;
      m_scale = fastuidraw::vecN<double, 2>(1.0, 1.0) / delta;
      m_scale *= static_cast<double>(CoordinateConverterConstants::box_dim);
      m_translate = pmin;
      m_delta_fudge = ::exp2(static_cast<double>(-CoordinateConverterConstants::negative_log2_fudge));
      m_scale_f = fastuidraw::vec2(m_scale);
      m_translate_f = fastuidraw::vec2(m_translate);
    }

    fastuidraw::vecN<double, 2>
    apply(const fastuidraw::vec2 &pt, unsigned int fudge_count) const
    {
      fastuidraw::vecN<double, 2> r, qt(pt);
      double fudge;

      r = m_scale * (qt - m_translate);
      fudge = static_cast<double>(fudge_count) * m_delta_fudge;
      r.x() += fudge;
      r.y() += fudge;
      return r;
    }

    fastuidraw::uvec2
    iapply(const fastuidraw::vec2 &pt) const
    {
      fastuidraw::vec2 r;
      fastuidraw::uvec2 return_value;

      r = m_scale_f * (pt - m_translate_f);

      return_value.x() = fastuidraw::t_max(0, static_cast<int>(r.x()));
      return_value.y() = fastuidraw::t_max(0, static_cast<int>(r.y()));

      return return_value;
    }

    double
    fudge_delta(void) const
    {
      return m_delta_fudge;
    }

  private:
    double m_delta_fudge;
    fastuidraw::vecN<double, 2> m_scale, m_translate;
    fastuidraw::vec2 m_scale_f, m_translate_f;
  };

  enum
    {
      box_max_x_flag = 1,
      box_max_y_flag = 2,
      box_min_x_min_y = 0 | 0,
      box_min_x_max_y = 0 | box_max_y_flag,
      box_max_x_max_y = box_max_x_flag | box_max_y_flag,
      box_max_x_min_y = box_max_x_flag,
    };

  class SubPath
  {
  public:
    typedef fastuidraw::vec2 SubContourPoint;
    typedef std::vector<SubContourPoint> SubContour;

    explicit
    SubPath(const fastuidraw::TessellatedPath &P);

    const std::vector<SubContour>&
    contours(void) const
    {
      return m_contours;
    }

    const fastuidraw::BoundingBox&
    bounds(void) const
    {
      return m_bounds;
    }

    unsigned int
    total_points(void) const
    {
      return m_total_points;
    }

    fastuidraw::vecN<SubPath*, 2>
    split(void) const;

  private:
    SubPath(const fastuidraw::BoundingBox &bb,
            std::vector<SubContour> &contours);

    int
    choose_splitting_coordinate(fastuidraw::vec2 mid_pt) const;

    static
    void
    copy_contour(SubContour &dst,
                 const fastuidraw::TessellatedPath &src, unsigned int C);

    static
    void
    split_contour(const SubContour &src,
                  int splitting_coordinate, float spitting_value,
                  SubContour &minC, SubContour &maxC);

    static
    fastuidraw::vec2
    compute_spit_point(fastuidraw::vec2 a,
                       fastuidraw::vec2 b,
                       int splitting_coordinate, float spitting_value);

    unsigned int m_total_points;
    fastuidraw::BoundingBox m_bounds;
    std::vector<SubContour> m_contours;
  };

  class PointHoard:fastuidraw::noncopyable
  {
  public:
    typedef std::vector<unsigned int> Contour;
    typedef std::list<Contour> Path;

    explicit
    PointHoard(const fastuidraw::BoundingBox &bounds,
               std::vector<fastuidraw::vec2> &pts):
      m_converter(bounds.min_point(), bounds.max_point()),
      m_pts(pts)
    {
      assert(!bounds.empty());
    }

    unsigned int
    fetch(const fastuidraw::vec2 &pt);

    void
    generate_path(const SubPath &input, Path &output);

    const fastuidraw::vec2&
    operator[](unsigned int v) const
    {
      assert(v < m_pts.size());
      return m_pts[v];
    }

    const fastuidraw::uvec2&
    ipt(unsigned int v) const
    {
      assert(v < m_ipts.size());
      return m_ipts[v];
    }

    const CoordinateConverter&
    converter(void) const
    {
      return m_converter;
    }

  private:
    void
    generate_contour(const SubPath::SubContour &input, Contour &output);

    CoordinateConverter m_converter;
    std::map<fastuidraw::uvec2, unsigned int> m_map;
    std::vector<fastuidraw::uvec2> m_ipts;
    std::vector<fastuidraw::vec2> &m_pts;
  };

  typedef fastuidraw::FilledPath::TriangleWithOppositeEdgeData AntiAliasedTriangle;

  class AntiAliasedTrianglesHoard:fastuidraw::noncopyable
  {
  public:
    ~AntiAliasedTrianglesHoard(void);

    void
    absorb(const AntiAliasedTrianglesHoard &obj);

    std::vector<AntiAliasedTriangle>&
    anti_aliased_triangles_writeable(int w);

    fastuidraw::const_c_array<AntiAliasedTriangle>
    anti_aliased_triangles(int w) const;

    unsigned int
    total_number_tris(void) const;

  private:
    /* anti-aliasing of data requires a seperate store for attribute
       and there is NO vertex sharing. The triangles for triangles
       with winding value W are in m_aa_painter_data[I] where
       I = FilledPath::chunk_from_winding_number(W)
     */
    std::vector<std::vector<AntiAliasedTriangle>* > m_data;
  };

  class Edge:public fastuidraw::uvec2
  {
  public:
    Edge(unsigned int a, unsigned int b):
      fastuidraw::uvec2(fastuidraw::t_min(a, b), fastuidraw::t_max(a, b))
    {}
  };

  class EdgeData
  {
  public:
    EdgeData(void):
      m_sorted(false)
    {}

    void
    add_winding(uint64_t area, int w, unsigned int v);

    /* Giving a vertex v, return the winding
       number of the triangle that shares the
       edge. If there are multiple choices
       for what vertex to choose that is not v,
       choose the vertex that:
        a) is on the opposite side of the edge
        b) maximizes area
     */
    int
    opposite_winding(int w, unsigned int v);

    class per_entry
    {
    public:
      uint64_t m_area;
      int m_winding;
      unsigned int m_vertex;

      /* NOTE: reverse sorted by AREA
       */
      bool
      operator<(const per_entry &rhs) const
      {
        return m_area > rhs.m_area;
      }

      bool
      canidate(int w, unsigned int v) const
      {
        return v != m_vertex || w != m_winding;
      }
    };

    std::vector<per_entry> m_entries;
    bool m_sorted;
  };

  /* class purpose is to track what triangle(s)
     and edge is used. Thse values are then used
     to compute AntiAliasedPoint::m_winding_opposite
   */
  class OppositeEdgeTracker:fastuidraw::noncopyable
  {
  public:
    explicit
    OppositeEdgeTracker(const PointHoard &points):
      m_points(points)
    {}

    void
    add_triangle(int w, uint64_t area,
                 unsigned int v0, unsigned int v1, unsigned int v2);

    void
    fill_aa_triangles(AntiAliasedTrianglesHoard &a,
                      const winding_index_hoard &raw_tris);

  private:
    void
    add_aa_triangle(std::vector<AntiAliasedTriangle> &dst,
                    int winding,
                    const fastuidraw::vecN<unsigned int, 3> &T);

    std::map<Edge, EdgeData> m_data;
    const PointHoard &m_points;
  };

  class tesser:fastuidraw::noncopyable
  {
  protected:
    explicit
    tesser(PointHoard &points);

    virtual
    ~tesser(void);

    void
    start(void);

    void
    stop(void);

    void
    add_path(const PointHoard::Path &P);

    void
    add_path_boundary(const SubPath &P);

    bool
    triangulation_failed(void)
    {
      return m_triangulation_failed;
    }

  private:

    virtual
    void
    on_begin_polygon(int winding_number) = 0;

    virtual
    void
    on_add_triangle(uint64_t area, unsigned int v0, unsigned int v1, unsigned int v2) = 0;

    virtual
    void
    add_vertex_to_polygon(unsigned int vertex) = 0;

    virtual
    FASTUIDRAW_GLUboolean
    fill_region(int winding_number) = 0;

    void
    add_contour(const PointHoard::Contour &C);

    static
    void
    begin_callBack(FASTUIDRAW_GLUenum type, int winding_number, void *tess);

    static
    void
    vertex_callBack(unsigned int vertex_data, void *tess);

    static
    void
    combine_callback(double x, double y, unsigned int data[4],
                     double weight[4],  unsigned int *outData,
                     void *tess);

    static
    FASTUIDRAW_GLUboolean
    winding_callBack(int winding_number, void *tess);

    unsigned int
    add_point_to_store(const fastuidraw::vec2 &p);

    bool
    temp_verts_non_degenerate_triangle(uint64_t &area);

    unsigned int m_point_count;
    fastuidraw_GLUtesselator *m_tess;
    PointHoard &m_points;
    fastuidraw::vecN<unsigned int, 3> m_temp_verts;
    unsigned int m_temp_vert_count;
    bool m_triangulation_failed;
  };

  class non_zero_tesser:private tesser
  {
  public:
    static
    bool
    execute_path(PointHoard &points,
                 const PointHoard::Path &P,
                 const SubPath &path,
                 winding_index_hoard &hoard,
                 OppositeEdgeTracker &opposite_edge_tracker)
    {
      non_zero_tesser NZ(points, P, path, hoard, opposite_edge_tracker);
      return NZ.triangulation_failed();
    }

  private:
    non_zero_tesser(PointHoard &points,
                    const PointHoard::Path &P,
                    const SubPath &path,
                    winding_index_hoard &hoard,
                    OppositeEdgeTracker &opposite_edge_tracker);

    virtual
    void
    on_begin_polygon(int winding_number);

    virtual
    void
    add_vertex_to_polygon(unsigned int vertex);

    virtual
    FASTUIDRAW_GLUboolean
    fill_region(int winding_number);

    virtual
    void
    on_add_triangle(uint64_t area, unsigned int v0, unsigned int v1, unsigned int v2);

    winding_index_hoard &m_hoard;
    OppositeEdgeTracker &m_opposite_edge_tracker;
    int m_current_winding;
    fastuidraw::reference_counted_ptr<per_winding_data> m_current_indices;
  };

  class zero_tesser:private tesser
  {
  public:
    static
    bool
    execute_path(PointHoard &points,
                 const PointHoard::Path &P,
                 const SubPath &path,
                 winding_index_hoard &hoard,
                 OppositeEdgeTracker &opposite_edge_tracker)
    {
      zero_tesser Z(points, P, path, hoard, opposite_edge_tracker);
      return Z.triangulation_failed();
    }

  private:

    zero_tesser(PointHoard &points,
                const PointHoard::Path &P,
                const SubPath &path,
                winding_index_hoard &hoard,
                OppositeEdgeTracker &opposite_edge_tracker);

    virtual
    void
    on_begin_polygon(int winding_number);

    virtual
    void
    add_vertex_to_polygon(unsigned int vertex);

    virtual
    FASTUIDRAW_GLUboolean
    fill_region(int winding_number);

    virtual
    void
    on_add_triangle(uint64_t area, unsigned int v0, unsigned int v1, unsigned int v2);

    fastuidraw::reference_counted_ptr<per_winding_data> &m_indices;
    OppositeEdgeTracker &m_opposite_edge_tracker;
  };

  class builder:fastuidraw::noncopyable
  {
  public:
    explicit
    builder(const SubPath &P, std::vector<fastuidraw::vec2> &pts,
            AntiAliasedTrianglesHoard &aa_triangles);

    ~builder();

    void
    fill_indices(std::vector<unsigned int> &indices,
                 std::map<int, fastuidraw::const_c_array<unsigned int> > &winding_map,
                 unsigned int &even_non_zero_start,
                 unsigned int &zero_start);

    bool
    triangulation_failed(void)
    {
      return m_failed;
    }

  private:
    winding_index_hoard m_hoard;
    PointHoard m_points;
    bool m_failed;
  };

  class AttributeDataMerger:public fastuidraw::PainterAttributeDataFiller
  {
  public:
    AttributeDataMerger(const fastuidraw::PainterAttributeData &a,
                        const fastuidraw::PainterAttributeData &b):
      m_a(a), m_b(b)
    {
    }

    virtual
    void
    compute_sizes(unsigned int &number_attributes,
                  unsigned int &number_indices,
                  unsigned int &number_attribute_chunks,
                  unsigned int &number_index_chunks,
                  unsigned int &number_z_increments) const;

    virtual
    void
    fill_data(fastuidraw::c_array<fastuidraw::PainterAttribute> attributes,
              fastuidraw::c_array<fastuidraw::PainterIndex> indices,
              fastuidraw::c_array<fastuidraw::const_c_array<fastuidraw::PainterAttribute> > attrib_chunks,
              fastuidraw::c_array<fastuidraw::const_c_array<fastuidraw::PainterIndex> > index_chunks,
              fastuidraw::c_array<unsigned int> zincrements,
              fastuidraw::c_array<int> index_adjusts) const;

    const fastuidraw::PainterAttributeData &m_a, &m_b;
  };

  class AttributeDataFiller:public fastuidraw::PainterAttributeDataFiller
  {
  public:
    std::vector<fastuidraw::vec2> m_points;

    /* Carefully organize indices as follows:
       - first all elements with odd winding number
       - then all elements with even and non-zero winding number
       - then all element with zero winding number.
       By doing so, the following are continuous in the array:
       - non-zero
       - odd-even fill rule
       - complement of odd-even fill
       - complement of non-zero
     */
    std::vector<unsigned int> m_indices;
    fastuidraw::const_c_array<unsigned int> m_nonzero_winding_indices;
    fastuidraw::const_c_array<unsigned int> m_zero_winding_indices;
    fastuidraw::const_c_array<unsigned int> m_odd_winding_indices;
    fastuidraw::const_c_array<unsigned int> m_even_winding_indices;

    /* m_per_fill[w] gives the indices to the triangles
       with the winding number w. The value points into
       indices
    */
    std::map<int, fastuidraw::const_c_array<unsigned int> > m_per_fill;

    virtual
    void
    compute_sizes(unsigned int &number_attributes,
                  unsigned int &number_indices,
                  unsigned int &number_attribute_chunks,
                  unsigned int &number_index_chunks,
                  unsigned int &number_z_increments) const;
    virtual
    void
    fill_data(fastuidraw::c_array<fastuidraw::PainterAttribute> attributes,
              fastuidraw::c_array<fastuidraw::PainterIndex> indices,
              fastuidraw::c_array<fastuidraw::const_c_array<fastuidraw::PainterAttribute> > attrib_chunks,
              fastuidraw::c_array<fastuidraw::const_c_array<fastuidraw::PainterIndex> > index_chunks,
              fastuidraw::c_array<unsigned int> zincrements,
              fastuidraw::c_array<int> index_adjusts) const;

    static
    fastuidraw::PainterAttribute
    generate_attribute(const fastuidraw::vec2 &src)
    {
      fastuidraw::PainterAttribute dst;

      dst.m_attrib0 = fastuidraw::pack_vec4(src.x(), src.y(), 0.0f, 0.0f);
      dst.m_attrib1 = fastuidraw::pack_vec4(1.0f, 1.0f, 1.0f, 0.0f);
      dst.m_attrib2 = fastuidraw::uvec4(0u, 0u, 0u, 0u);

      return dst;
    }
  };

  class ScratchSpacePrivate
  {
  public:
    std::vector<fastuidraw::vec3> m_adjusted_clip_eqs;
    std::vector<fastuidraw::vec2> m_clipped_rect;

    fastuidraw::vecN<std::vector<fastuidraw::vec2>, 2> m_clip_scratch_vec2s;
    std::vector<float> m_clip_scratch_floats;
  };

  class SubsetPrivate
  {
  public:
    ~SubsetPrivate(void);

    SubsetPrivate(SubPath *P, int max_recursion,
                  std::vector<SubsetPrivate*> &out_values);

    unsigned int
    select_subsets(ScratchSpacePrivate &scratch,
                   bool with_anti_aliasing,
                   fastuidraw::const_c_array<fastuidraw::vec3> clip_equations,
                   const fastuidraw::float3x3 &clip_matrix_local,
                   unsigned int max_attribute_cnt,
                   unsigned int max_index_cnt,
                   fastuidraw::c_array<unsigned int> dst);

    void
    make_ready(void);

    fastuidraw::const_c_array<int>
    winding_numbers(void)
    {
      assert(m_painter_data != NULL);
      return fastuidraw::make_c_array(m_winding_numbers);
    }

    const fastuidraw::PainterAttributeData&
    painter_data(void) const
    {
      assert(m_painter_data != NULL);
      return *m_painter_data;
    }

    const AntiAliasedTrianglesHoard&
    aa_triangles(void) const
    {
      assert(m_painter_data != NULL);
      return m_aa_triangles;
    }

  private:
    bool
    can_fit_into_buffer(bool use_aa_triangles,
                        unsigned int max_attribute_cnt,
                        unsigned int max_index_cnt)
    {
      assert(m_sizes_ready);
      return use_aa_triangles ?
        m_num_attributes <= max_attribute_cnt && m_largest_index_block <= max_index_cnt :
        m_num_aa_triangle_points <= max_attribute_cnt && m_num_aa_triangle_points <= max_index_cnt;
    }

    void
    select_subsets_implement(ScratchSpacePrivate &scratch,
                             bool with_anti_aliasing,
                             fastuidraw::c_array<unsigned int> dst,
                             unsigned int max_attribute_cnt,
                             unsigned int max_index_cnt,
                             unsigned int &current);

    void
    select_subsets_all_unculled(fastuidraw::c_array<unsigned int> dst,
                                bool with_anti_aliasing,
                                unsigned int max_attribute_cnt,
                                unsigned int max_index_cnt,
                                unsigned int &current);

    void
    make_ready_from_children(void);

    void
    make_ready_from_sub_path(void);

    /* m_ID represents an index into the std::vector<>
       passed into create_hierarchy() where this element
       is found.
     */
    unsigned int m_ID;

    /* The bounds of this SubsetPrivate used in
       select_subsets().
     */
    fastuidraw::BoundingBox m_bounds;

    /* if this SubsetPrivate has children then
       m_painter_data is made by "merging" the
       data of m_painter_data from m_children[0]
       and m_children[1]. We do this merging so
       that we can avoid recursing if the entirity
       of the bounding box is contained in the
       clipping region.
     */
    fastuidraw::PainterAttributeData *m_painter_data;
    std::vector<int> m_winding_numbers;
    AntiAliasedTrianglesHoard m_aa_triangles;

    bool m_sizes_ready;
    unsigned int m_num_attributes;
    unsigned int m_largest_index_block;
    unsigned int m_num_aa_triangle_points;

    /* m_sub_path is non-NULL only if this SubsetPrivate
       has no children. In addition, it is set to NULL
       and deleted when m_painter_data is created from
       it.
     */
    SubPath *m_sub_path;
    fastuidraw::vecN<SubsetPrivate*, 2> m_children;
  };

  class DataWriterPrivate
  {
  public:
    DataWriterPrivate(void):
      m_with_anti_aliasing(false)
    {}

    class per_index_chunk
    {
    public:
      explicit
      per_index_chunk(fastuidraw::const_c_array<fastuidraw::PainterIndex> indices,
                      unsigned int attrib_chunk):
        m_indices(indices),
        m_attrib_chunk(attrib_chunk)
      {}

      fastuidraw::const_c_array<fastuidraw::PainterIndex> m_indices;
      unsigned int m_attrib_chunk;
    };

    class per_attrib_chunk
    {
    public:
      explicit
      per_attrib_chunk(const SubsetPrivate *d):
        m_attribs(d->painter_data().attribute_data_chunk(0))
      {}

      fastuidraw::const_c_array<fastuidraw::PainterAttribute> m_attribs;
    };

    std::vector<unsigned int> m_subset_selector;
    std::vector<per_attrib_chunk> m_attribute_chunks;
    std::vector<per_index_chunk> m_index_chunks;
    bool m_with_anti_aliasing;
  };

  class FilledPathPrivate
  {
  public:
    explicit
    FilledPathPrivate(const fastuidraw::TessellatedPath &P);

    ~FilledPathPrivate();

    SubsetPrivate *m_root;
    std::vector<SubsetPrivate*> m_subsets;
  };
}

/////////////////////////////////////
// SubPath methods
SubPath::
SubPath(const fastuidraw::BoundingBox &bb,
        std::vector<SubContour> &contours):
  m_total_points(0),
  m_bounds(bb)
{
  m_contours.swap(contours);
  for(std::vector<SubContour>::const_iterator c_iter = m_contours.begin(),
        c_end = m_contours.end(); c_iter != c_end; ++c_iter)
    {
      m_total_points += c_iter->size();
    }
}

SubPath::
SubPath(const fastuidraw::TessellatedPath &P):
  m_total_points(0),
  m_bounds(P.bounding_box_min(),
           P.bounding_box_max()),
  m_contours(P.number_contours())
{
  for(unsigned int c = 0, endc = m_contours.size(); c < endc; ++c)
    {
      copy_contour(m_contours[c], P, c);
      m_total_points += m_contours[c].size();
    }
}

void
SubPath::
copy_contour(SubContour &dst,
             const fastuidraw::TessellatedPath &src, unsigned int C)
{
  for(unsigned int e = 0, ende = src.number_edges(C); e < ende; ++e)
    {
      fastuidraw::range_type<unsigned int> R;

      R = src.edge_range(C, e);
      for(unsigned int v = R.m_begin; v + 1 < R.m_end; ++v)
        {
          dst.push_back(src.point_data()[v].m_p);
        }
    }
}

int
SubPath::
choose_splitting_coordinate(fastuidraw::vec2 mid_pt) const
{
  /* do not allow the box to be too far from being a square.
     TODO: if the balance of points heavily favors the other
     side, we should ignore the size_max_ratio. Perhaps a
     wieght factor between the different in # of points
     of the sides and the ratio?
   */
  if(SubsetConstants::size_max_ratio > 0.0f)
    {
      fastuidraw::vec2 wh;
      wh = m_bounds.max_point() - m_bounds.min_point();
      if(wh.x() >= SubsetConstants::size_max_ratio * wh.y())
        {
          return 0;
        }
      else if(wh.y() >= SubsetConstants::size_max_ratio * wh.x())
        {
          return 1;
        }
    }

  /* first find which of splitting in X or splitting in Y
     is optimal.
   */
  fastuidraw::ivec2 number_points_before(0, 0);
  fastuidraw::ivec2 number_points_after(0, 0);
  fastuidraw::ivec2 number_points;

  for(std::vector<SubContour>::const_iterator c_iter = m_contours.begin(),
        c_end = m_contours.end(); c_iter != c_end; ++c_iter)
    {
      fastuidraw::vec2 prev_pt(c_iter->back());
      for(SubContour::const_iterator iter = c_iter->begin(),
            end = c_iter->end(); iter != end; ++iter)
        {
          fastuidraw::vec2 pt(*iter);
          for(int i = 0; i < 2; ++i)
            {
              bool prev_b, b;

              prev_b = prev_pt[i] < mid_pt[i];
              b = pt[i] < mid_pt[i];

              if(b || pt[i] == mid_pt[i])
                {
                  ++number_points_before[i];
                }

              if(!b || pt[i] == mid_pt[i])
                {
                  ++number_points_after[i];
                }

              if(prev_pt[i] != mid_pt[i] && prev_b != b)
                {
                  ++number_points_before[i];
                  ++number_points_after[i];
                }
            }
          prev_pt = pt;
        }
    }

  /* choose a splitting that:
      - minimizes number_points_before[i] + number_points_after[i]
   */
  number_points = number_points_before + number_points_after;
  if(number_points.x() < number_points.y())
    {
      return 0;
    }
  else
    {
      return 1;
    }
}

fastuidraw::vec2
SubPath::
compute_spit_point(fastuidraw::vec2 a, fastuidraw::vec2 b,
                   int splitting_coordinate, float splitting_value)
{
  float t, n, d, aa, bb;
  fastuidraw::vec2 return_value;

  n = splitting_value - a[splitting_coordinate];
  d = b[splitting_coordinate] - a[splitting_coordinate];
  t = n / d;

  return_value[splitting_coordinate] = splitting_value;

  aa = a[1 - splitting_coordinate];
  bb = b[1 - splitting_coordinate];
  return_value[1 - splitting_coordinate] = (1.0f - t) * aa + t * bb;

  return return_value;
}

void
SubPath::
split_contour(const SubContour &src,
              int splitting_coordinate, float splitting_value,
              SubContour &C0, SubContour &C1)
{
  SubContourPoint prev_pt(src.back());
  for(SubContour::const_iterator iter = src.begin(),
        end = src.end(); iter != end; ++iter)
    {
      bool b0, prev_b0;
      bool b1, prev_b1;
      fastuidraw::vec2 split_pt;
      const SubContourPoint &pt(*iter);

      prev_b0 = prev_pt[splitting_coordinate] <= splitting_value;
      b0 = pt[splitting_coordinate] <= splitting_value;

      prev_b1 = prev_pt[splitting_coordinate] >= splitting_value;
      b1 = pt[splitting_coordinate] >= splitting_value;

      if(prev_b0 != b0 || prev_b1 != b1)
        {
          split_pt = compute_spit_point(prev_pt, pt,
                                        splitting_coordinate, splitting_value);
        }

      if(prev_b0 != b0)
        {
          C0.push_back(split_pt);
        }

      if(b0)
        {
          C0.push_back(pt);
        }

      if(prev_b1 != b1)
        {
          C1.push_back(split_pt);
        }

      if(b1)
        {
          C1.push_back(pt);
        }

      prev_pt = pt;
    }
}

fastuidraw::vecN<SubPath*, 2>
SubPath::
split(void) const
{
  fastuidraw::vecN<SubPath*, 2> return_value(NULL, NULL);
  fastuidraw::vec2 mid_pt;
  int splitting_coordinate;

  mid_pt = 0.5f * (m_bounds.max_point() + m_bounds.min_point());
  splitting_coordinate = choose_splitting_coordinate(mid_pt);

  /* now split each contour.
   */
  fastuidraw::vec2 B0_max, B1_min;
  B0_max[1 - splitting_coordinate] = m_bounds.max_point()[1 - splitting_coordinate];
  B0_max[splitting_coordinate] = mid_pt[splitting_coordinate];

  B1_min[1 - splitting_coordinate] = m_bounds.min_point()[1 - splitting_coordinate];
  B1_min[splitting_coordinate] = mid_pt[splitting_coordinate];

  fastuidraw::BoundingBox B0(m_bounds.min_point(), B0_max);
  fastuidraw::BoundingBox B1(B1_min, m_bounds.max_point());
  std::vector<SubContour> C0, C1;

  C0.reserve(m_contours.size());
  C1.reserve(m_contours.size());
  for(std::vector<SubContour>::const_iterator c_iter = m_contours.begin(),
        c_end = m_contours.end(); c_iter != c_end; ++c_iter)
    {
      C0.push_back(SubContour());
      C1.push_back(SubContour());
      split_contour(*c_iter, splitting_coordinate,
                    mid_pt[splitting_coordinate],
                    C0.back(), C1.back());

      if(C0.back().empty())
        {
          C0.pop_back();
        }

      if(C1.back().empty())
        {
          C1.pop_back();
        }
    }

  return_value[0] = FASTUIDRAWnew SubPath(B0, C0);
  return_value[1] = FASTUIDRAWnew SubPath(B1, C1);

  return return_value;
}

//////////////////////////////////////
// PointHoard methods
unsigned int
PointHoard::
fetch(const fastuidraw::vec2 &pt)
{
  std::map<fastuidraw::uvec2, unsigned int>::iterator iter;
  fastuidraw::uvec2 ipt;
  unsigned int return_value;

  assert(m_pts.size() == m_ipts.size());

  ipt = m_converter.iapply(pt);
  iter = m_map.find(ipt);
  if(iter != m_map.end())
    {
      return_value = iter->second;
    }
  else
    {
      return_value = m_pts.size();
      m_pts.push_back(pt);
      m_ipts.push_back(ipt);
      m_map[ipt] = return_value;
    }
  return return_value;
}

void
PointHoard::
generate_path(const SubPath &input, Path &output)
{
  output.clear();
  const std::vector<SubPath::SubContour> &contours(input.contours());
  for(std::vector<SubPath::SubContour>::const_iterator iter = contours.begin(),
        end = contours.end(); iter != end; ++iter)
    {
      const SubPath::SubContour &C(*iter);
      output.push_back(Contour());
      generate_contour(C, output.back());
    }
}

void
PointHoard::
generate_contour(const SubPath::SubContour &C, Contour &output)
{
  unsigned int total_cnt(0), cnt(0);

  for(unsigned int v = 0, endv = C.size(); v < endv; ++v,  ++cnt, ++total_cnt)
    {
      output.push_back(fetch(C[v]));
    }
}

////////////////////////////////////////
// tesser methods
tesser::
tesser(PointHoard &points):
  m_point_count(0),
  m_points(points),
  m_triangulation_failed(false)
{
  m_tess = fastuidraw_gluNewTess;
  fastuidraw_gluTessCallbackBegin(m_tess, &begin_callBack);
  fastuidraw_gluTessCallbackVertex(m_tess, &vertex_callBack);
  fastuidraw_gluTessCallbackCombine(m_tess, &combine_callback);
  fastuidraw_gluTessCallbackFillRule(m_tess, &winding_callBack);
  fastuidraw_gluTessPropertyBoundaryOnly(m_tess, FASTUIDRAW_GLU_FALSE);
}

tesser::
~tesser(void)
{
  fastuidraw_gluDeleteTess(m_tess);
}


void
tesser::
start(void)
{
  fastuidraw_gluTessBeginPolygon(m_tess, this);
}

void
tesser::
stop(void)
{
  fastuidraw_gluTessEndPolygon(m_tess);
}

void
tesser::
add_path(const PointHoard::Path &P)
{
  for(PointHoard::Path::const_iterator iter = P.begin(),
        end = P.end(); iter != end; ++iter)
    {
      add_contour(*iter);
    }
}

void
tesser::
add_contour(const PointHoard::Contour &C)
{
  fastuidraw_gluTessBeginContour(m_tess, FASTUIDRAW_GLU_TRUE);
  for(unsigned int v = 0, endv = C.size(); v < endv; ++v)
    {
      fastuidraw::vecN<double, 2> p;
      unsigned int I;

      /* TODO: Incrementing the amount by which to apply
         fudge is not the correct thing to do. Rather, we
         should only increment and apply fudge on overlapping
         and degenerate edges.
      */
      I = C[v];
      p = m_points.converter().apply(m_points[I], m_point_count);
      ++m_point_count;

      fastuidraw_gluTessVertex(m_tess, p.x(), p.y(), I);
    }
  fastuidraw_gluTessEndContour(m_tess);
}

void
tesser::
add_path_boundary(const SubPath &P)
{
  fastuidraw::vec2 pmin, pmax;
  unsigned int src[4] =
    {
      box_min_x_min_y,
      box_min_x_max_y,
      box_max_x_max_y,
      box_max_x_min_y,
    };

  pmin = P.bounds().min_point();
  pmax = P.bounds().max_point();

  fastuidraw_gluTessBeginContour(m_tess, FASTUIDRAW_GLU_TRUE);
  for(unsigned int i = 0; i < 4; ++i)
    {
      double slack, x, y;
      unsigned int k;
      fastuidraw::vec2 p;

      slack = static_cast<double>(m_point_count) * m_points.converter().fudge_delta();
      k = src[i];
      if(k & box_max_x_flag)
        {
          x = slack + static_cast<double>(CoordinateConverterConstants::box_dim);
          p.x() = pmax.x();
        }
      else
        {
          x = -slack;
          p.x() = pmin.x();
        }

      if(k & box_max_y_flag)
        {
          y = slack + static_cast<double>(CoordinateConverterConstants::box_dim);
          p.y() = pmax.y();
        }
      else
        {
          y = -slack;
          p.y() = pmin.y();
        }
      fastuidraw_gluTessVertex(m_tess, x, y, m_points.fetch(p));
    }
  fastuidraw_gluTessEndContour(m_tess);
}

unsigned int
tesser::
add_point_to_store(const fastuidraw::vec2 &p)
{
  unsigned int return_value;
  return_value = m_points.fetch(p);
  return return_value;
}

bool
tesser::
temp_verts_non_degenerate_triangle(uint64_t &area)
{
  if(m_temp_verts[0] == m_temp_verts[1]
     || m_temp_verts[0] == m_temp_verts[2]
     || m_temp_verts[1] == m_temp_verts[2])
    {
      return false;
    }

  fastuidraw::i64vec2 p0(m_points.ipt(m_temp_verts[0]));
  fastuidraw::i64vec2 p1(m_points.ipt(m_temp_verts[1]));
  fastuidraw::i64vec2 p2(m_points.ipt(m_temp_verts[2]));

  if(p0 == p1 || p0 == p2 || p1 == p2)
    {
      return false;
    }

  fastuidraw::i64vec2 v(p1 - p0), w(p2 - p0);
  bool return_value;

  /* we only reject a triangle if its area is zero.
   */
  area = fastuidraw::t_abs(v.x() * w.y() - v.y() * w.x());
  return_value = (area > 0);
  return return_value;
}

void
tesser::
begin_callBack(FASTUIDRAW_GLUenum type, int winding_number, void *tess)
{
  tesser *p;
  p = static_cast<tesser*>(tess);
  assert(FASTUIDRAW_GLU_TRIANGLES == type);
  FASTUIDRAWunused(type);

  p->m_temp_vert_count = 0;
  p->on_begin_polygon(winding_number);
}

void
tesser::
vertex_callBack(unsigned int vertex_id, void *tess)
{
  tesser *p;
  p = static_cast<tesser*>(tess);

  if(vertex_id == FASTUIDRAW_GLU_NULL_CLIENT_ID)
    {
      p->m_triangulation_failed = true;
    }

  /* Cache adds vertices in groups of 3 (triangles),
     then if all vertices are NOT FASTUIDRAW_GLU_NULL_CLIENT_ID,
     and triangle is not degenerate, then add the triangle.
   */
  p->m_temp_verts[p->m_temp_vert_count] = vertex_id;
  p->m_temp_vert_count++;
  if(p->m_temp_vert_count == 3)
    {
      uint64_t area(0u);
      p->m_temp_vert_count = 0;
      /*
        if a vertex_id is FASTUIDRAW_GLU_NULL_CLIENT_ID, that means
        the triangle is junked.
      */
      if(p->m_temp_verts[0] != FASTUIDRAW_GLU_NULL_CLIENT_ID
         && p->m_temp_verts[1] != FASTUIDRAW_GLU_NULL_CLIENT_ID
         && p->m_temp_verts[2] != FASTUIDRAW_GLU_NULL_CLIENT_ID
         && p->temp_verts_non_degenerate_triangle(area))
        {
          p->add_vertex_to_polygon(p->m_temp_verts[0]);
          p->add_vertex_to_polygon(p->m_temp_verts[1]);
          p->add_vertex_to_polygon(p->m_temp_verts[2]);
          p->on_add_triangle(area, p->m_temp_verts[0], p->m_temp_verts[1], p->m_temp_verts[2]);
        }
    }
}

void
tesser::
combine_callback(double x, double y, unsigned int data[4],
                 double weight[4],  unsigned int *outData,
                 void *tess)
{
  FASTUIDRAWunused(x);
  FASTUIDRAWunused(y);

  tesser *p;
  unsigned int v;
  fastuidraw::vec2 pt(0.0f, 0.0f);

  p = static_cast<tesser*>(tess);
  for(unsigned int i = 0; i < 4; ++i)
    {
      if(data[i] != FASTUIDRAW_GLU_NULL_CLIENT_ID)
        {
          pt += float(weight[i]) * p->m_points[data[i]];
        }
    }
  v = p->add_point_to_store(pt);
  *outData = v;
}

FASTUIDRAW_GLUboolean
tesser::
winding_callBack(int winding_number, void *tess)
{
  tesser *p;
  FASTUIDRAW_GLUboolean return_value;

  p = static_cast<tesser*>(tess);
  return_value = p->fill_region(winding_number);
  return return_value;
}

///////////////////////////////////
// non_zero_tesser methods
non_zero_tesser::
non_zero_tesser(PointHoard &points,
                const PointHoard::Path &P,
                const SubPath &path,
                winding_index_hoard &hoard,
                OppositeEdgeTracker &opposite_edge_tracker):
  tesser(points),
  m_hoard(hoard),
  m_opposite_edge_tracker(opposite_edge_tracker),
  m_current_winding(0)
{
  FASTUIDRAWunused(path);
  start();
  add_path(P);
  stop();
}

void
non_zero_tesser::
on_begin_polygon(int winding_number)
{
  if(!m_current_indices || m_current_winding != winding_number)
    {
      fastuidraw::reference_counted_ptr<per_winding_data> &h(m_hoard[winding_number]);
      m_current_winding = winding_number;
      if(!h)
        {
          h = FASTUIDRAWnew per_winding_data();
        }
      m_current_indices = h;
    }
}

void
non_zero_tesser::
on_add_triangle(uint64_t area, unsigned int v0, unsigned int v1, unsigned int v2)
{
  m_opposite_edge_tracker.add_triangle(m_current_winding, area, v0, v1, v2);
}

void
non_zero_tesser::
add_vertex_to_polygon(unsigned int vertex)
{
  m_current_indices->add_index(vertex);
}


FASTUIDRAW_GLUboolean
non_zero_tesser::
fill_region(int winding_number)
{
  return winding_number != 0 ?
    FASTUIDRAW_GLU_TRUE :
    FASTUIDRAW_GLU_FALSE;
}

///////////////////////////////
// zero_tesser methods
zero_tesser::
zero_tesser(PointHoard &points,
            const PointHoard::Path &P,
            const SubPath &path,
            winding_index_hoard &hoard,
            OppositeEdgeTracker &opposite_edge_tracker):
  tesser(points),
  m_indices(hoard[0]),
  m_opposite_edge_tracker(opposite_edge_tracker)
{
  if(!m_indices)
    {
      m_indices = FASTUIDRAWnew per_winding_data();
    }

  start();
  add_path(P);
  add_path_boundary(path);
  stop();
}

void
zero_tesser::
on_begin_polygon(int winding_number)
{
  assert(winding_number == -1);
  FASTUIDRAWunused(winding_number);
}

void
zero_tesser::
on_add_triangle(uint64_t area, unsigned int v0, unsigned int v1, unsigned int v2)
{
  m_opposite_edge_tracker.add_triangle(0, area, v0, v1, v2);
}

void
zero_tesser::
add_vertex_to_polygon(unsigned int vertex)
{
  m_indices->add_index(vertex);
}

FASTUIDRAW_GLUboolean
zero_tesser::
fill_region(int winding_number)
{
  return winding_number == -1 ?
    FASTUIDRAW_GLU_TRUE :
    FASTUIDRAW_GLU_FALSE;
}

/////////////////////////////////////////
// builder methods
builder::
builder(const SubPath &P, std::vector<fastuidraw::vec2> &points,
        AntiAliasedTrianglesHoard &aa_triangles):
  m_points(P.bounds(), points)
{
  bool failZ, failNZ;
  PointHoard::Path path;
  OppositeEdgeTracker oet(m_points);

  m_points.generate_path(P, path);
  failNZ = non_zero_tesser::execute_path(m_points, path, P, m_hoard, oet);
  failZ = zero_tesser::execute_path(m_points, path, P, m_hoard, oet);
  oet.fill_aa_triangles(aa_triangles, m_hoard);
  m_failed = failNZ || failZ;
}

builder::
~builder()
{
}

void
builder::
fill_indices(std::vector<unsigned int> &indices,
             std::map<int, fastuidraw::const_c_array<unsigned int> > &winding_map,
             unsigned int &even_non_zero_start,
             unsigned int &zero_start)
{
  winding_index_hoard::iterator iter, end;
  unsigned int total(0), num_odd(0), num_even_non_zero(0), num_zero(0);

  /* compute number indices needed */
  for(iter = m_hoard.begin(), end = m_hoard.end(); iter != end; ++iter)
    {
      unsigned int cnt;

      cnt = iter->second->count();
      total += cnt;
      if(iter->first == 0)
        {
          num_zero += cnt;
        }
      else if (is_even(iter->first))
        {
          num_even_non_zero += cnt;
        }
      else
        {
          num_odd += cnt;
        }
    }

  /* pack as follows:
      - odd
      - even non-zero
      - zero
   */
  unsigned int current_odd(0), current_even_non_zero(num_odd);
  unsigned int current_zero(num_even_non_zero + num_odd);

  indices.resize(total);
  for(iter = m_hoard.begin(), end = m_hoard.end(); iter != end; ++iter)
    {
      if(iter->first == 0)
        {
          if(iter->second->count() > 0)
            {
              iter->second->fill_at(current_zero,
                                    fastuidraw::make_c_array(indices),
                                    winding_map[iter->first]);
            }
        }
      else if(is_even(iter->first))
        {
          if(iter->second->count() > 0)
            {
              iter->second->fill_at(current_even_non_zero,
                                    fastuidraw::make_c_array(indices),
                                    winding_map[iter->first]);
            }
        }
      else
        {
          if(iter->second->count() > 0)
            {
              iter->second->fill_at(current_odd,
                                    fastuidraw::make_c_array(indices),
                                    winding_map[iter->first]);
            }
        }
    }

  assert(current_zero == total);
  assert(current_odd == num_odd);
  assert(current_even_non_zero == current_odd + num_even_non_zero);

  even_non_zero_start = num_odd;
  zero_start = current_odd + num_even_non_zero;
}

////////////////////////////////
// AttributeDataMerger methods
void
AttributeDataMerger::
compute_sizes(unsigned int &number_attributes,
              unsigned int &number_indices,
              unsigned int &number_attribute_chunks,
              unsigned int &number_index_chunks,
              unsigned int &number_z_increments) const
{
  number_z_increments = 0;
  number_attributes = m_a.attribute_data_chunk(0).size() + m_b.attribute_data_chunk(0).size();
  number_attribute_chunks = 1;
  number_index_chunks = fastuidraw::t_max(m_a.index_data_chunks().size(),
                                          m_b.index_data_chunks().size());
  number_indices = 0;
  for(unsigned int c = 0; c < number_index_chunks; ++c)
    {
      unsigned int a_sz, b_sz;

      a_sz = m_a.index_data_chunk(c).size();
      b_sz = m_b.index_data_chunk(c).size();
      number_indices += (a_sz + b_sz);
    }
}


void
AttributeDataMerger::
fill_data(fastuidraw::c_array<fastuidraw::PainterAttribute> attributes,
          fastuidraw::c_array<fastuidraw::PainterIndex> indices,
          fastuidraw::c_array<fastuidraw::const_c_array<fastuidraw::PainterAttribute> > attrib_chunks,
          fastuidraw::c_array<fastuidraw::const_c_array<fastuidraw::PainterIndex> > index_chunks,
          fastuidraw::c_array<unsigned int> zincrements,
          fastuidraw::c_array<int> index_adjusts) const
{
  fastuidraw::c_array<fastuidraw::PainterAttribute> a_attribs, b_attribs;

  FASTUIDRAWunused(zincrements);

  a_attribs = attributes.sub_array(0, m_a.attribute_data_chunk(0).size());
  b_attribs = attributes.sub_array(m_a.attribute_data_chunk(0).size());
  assert(b_attribs.size() == m_b.attribute_data_chunk(0).size());

  attrib_chunks[0] = attributes;
  /* copy attributes with attributes of m_a first
   */
  std::copy(m_a.attribute_data_chunk(0).begin(),
            m_a.attribute_data_chunk(0).end(),
            a_attribs.begin());

  std::copy(m_b.attribute_data_chunk(0).begin(),
            m_b.attribute_data_chunk(0).end(),
            b_attribs.begin());

  /* copy indices is trickier; we need to copy with correct chunking
     AND adjust the values for the indices coming from m_b (because
     m_b attributes are placed after m_a attributes).
   */
  for(unsigned int chunk = 0, end_chunk = index_chunks.size(), current = 0; chunk < end_chunk; ++chunk)
    {
      fastuidraw::c_array<fastuidraw::PainterIndex> dst, dst_a, dst_b;
      unsigned int dst_size, a_sz, b_sz;

      index_adjusts[chunk] = 0;

      a_sz = m_a.index_data_chunk(chunk).size();
      b_sz = m_b.index_data_chunk(chunk).size();
      dst_size = a_sz + b_sz;

      dst = indices.sub_array(current, dst_size);
      index_chunks[chunk] = dst;
      dst_a = dst.sub_array(0, a_sz);
      dst_b = dst.sub_array(a_sz);
      current += dst_size;

      if(a_sz > 0)
        {
          std::copy(m_a.index_data_chunk(chunk).begin(),
                    m_a.index_data_chunk(chunk).end(),
                    dst_a.begin());
        }

      if(b_sz > 0)
        {
          fastuidraw::const_c_array<fastuidraw::PainterIndex> src;

          src = m_b.index_data_chunk(chunk);
          for(unsigned int i = 0; i < b_sz; ++i)
            {
              dst_b[i] = src[i] + a_attribs.size();
            }
        }
    }
}

////////////////////////////////////
// AttributeDataFiller methods
void
AttributeDataFiller::
compute_sizes(unsigned int &number_attributes,
              unsigned int &number_indices,
              unsigned int &number_attribute_chunks,
              unsigned int &number_index_chunks,
              unsigned int &number_z_increments) const
{
  using namespace fastuidraw;

  number_z_increments = 0;
  if(m_per_fill.empty())
    {
      number_attributes = 0;
      number_indices = 0;
      number_attribute_chunks = 0;
      number_index_chunks = 0;
      return;
    }
  number_attributes = m_points.size();
  number_attribute_chunks = 1;

  number_indices = m_odd_winding_indices.size()
    + m_nonzero_winding_indices.size()
    + m_even_winding_indices.size()
    + m_zero_winding_indices.size();

  for(std::map<int, const_c_array<unsigned int> >::const_iterator
        iter = m_per_fill.begin(), end = m_per_fill.end();
      iter != end; ++iter)
    {
      if(iter->first != 0) //winding number 0 is by complement_nonzero_fill_rule
        {
          number_indices += iter->second.size();
        }
    }

  /* now get how big the index_chunks really needs to be
   */
  int smallest_winding(m_per_fill.begin()->first);
  int largest_winding(m_per_fill.rbegin()->first);
  unsigned int largest_winding_idx(FilledPath::Subset::chunk_from_winding_number(largest_winding));
  unsigned int smallest_winding_idx(FilledPath::Subset::chunk_from_winding_number(smallest_winding));
  number_index_chunks = 1 + std::max(largest_winding_idx, smallest_winding_idx);
}

void
AttributeDataFiller::
fill_data(fastuidraw::c_array<fastuidraw::PainterAttribute> attributes,
          fastuidraw::c_array<fastuidraw::PainterIndex> index_data,
          fastuidraw::c_array<fastuidraw::const_c_array<fastuidraw::PainterAttribute> > attrib_chunks,
          fastuidraw::c_array<fastuidraw::const_c_array<fastuidraw::PainterIndex> > index_chunks,
          fastuidraw::c_array<unsigned int> zincrements,
          fastuidraw::c_array<int> index_adjusts) const
{
  using namespace fastuidraw;

  if(m_per_fill.empty())
    {
      return;
    }
  assert(attributes.size() == m_points.size());
  assert(attrib_chunks.size() == 1);
  assert(zincrements.empty());
  FASTUIDRAWunused(zincrements);

  /* generate attribute data
   */
  std::transform(m_points.begin(), m_points.end(), attributes.begin(),
                 AttributeDataFiller::generate_attribute);
  attrib_chunks[0] = attributes;
  std::fill(index_adjusts.begin(), index_adjusts.end(), 0);

  unsigned int current(0);

#define GRAB_MACRO(enum_name, member_name) do {                     \
    c_array<PainterIndex> dst;                                      \
    dst = index_data.sub_array(current, member_name.size());        \
    std::copy(member_name.begin(),                                  \
              member_name.end(), dst.begin());                      \
    index_chunks[PainterEnums::enum_name] = dst;                    \
    current += dst.size();                                          \
  } while(0)

  GRAB_MACRO(odd_even_fill_rule, m_odd_winding_indices);
  GRAB_MACRO(nonzero_fill_rule, m_nonzero_winding_indices);
  GRAB_MACRO(complement_odd_even_fill_rule, m_even_winding_indices);
  GRAB_MACRO(complement_nonzero_fill_rule, m_zero_winding_indices);

#undef GRAB_MACRO

  for(std::map<int, const_c_array<unsigned int> >::const_iterator
        iter = m_per_fill.begin(), end = m_per_fill.end();
      iter != end; ++iter)
    {
      if(iter->first != 0) //winding number 0 is by complement_nonzero_fill_rule
        {
          c_array<PainterIndex> dst;
          const_c_array<unsigned int> src;
          unsigned int idx;

          idx = FilledPath::Subset::chunk_from_winding_number(iter->first);

          src = iter->second;
          dst = index_data.sub_array(current, src.size());
          assert(dst.size() == src.size());

          std::copy(src.begin(), src.end(), dst.begin());

          index_chunks[idx] = dst;
          current += dst.size();
        }
    }
}

///////////////////////////////////
// AntiAliasedTrianglesHoard methods
AntiAliasedTrianglesHoard::
~AntiAliasedTrianglesHoard(void)
{
  for(unsigned int i = 0, endi = m_data.size(); i < endi; ++i)
    {
      FASTUIDRAWdelete(m_data[i]);
    }
}

std::vector<AntiAliasedTriangle>&
AntiAliasedTrianglesHoard::
anti_aliased_triangles_writeable(int w)
{
  unsigned int I;
  I = fastuidraw::FilledPath::Subset::chunk_from_winding_number(w);
  if(I >= m_data.size())
    {
      unsigned int oldSize;
      oldSize = m_data.size();
      m_data.resize(I + 1, NULL);
      for(unsigned int i = oldSize; i <= I; ++i)
        {
          m_data[i] = FASTUIDRAWnew std::vector<AntiAliasedTriangle>();
        }
    }
  assert(I < m_data.size() && m_data[I] != NULL);
  return *m_data[I];
}

fastuidraw::const_c_array<AntiAliasedTriangle>
AntiAliasedTrianglesHoard::
anti_aliased_triangles(int w) const
{
  unsigned int I;

  I = fastuidraw::FilledPath::Subset::chunk_from_winding_number(w);
  assert(I >= m_data.size() || m_data[I] != NULL);
  return I < m_data.size() ?
    fastuidraw::make_c_array(*m_data[I]) :
    fastuidraw::const_c_array<AntiAliasedTriangle>();
}

unsigned int
AntiAliasedTrianglesHoard::
total_number_tris(void) const
{
  unsigned int return_value;
  for(unsigned int i = 0, endi = m_data.size(); i < endi; ++i)
    {
      if(m_data[i] != NULL)
        {
          return_value += m_data[i]->size();
        }
    }
  return return_value;
}

void
AntiAliasedTrianglesHoard::
absorb(const AntiAliasedTrianglesHoard &obj)
{
  unsigned int sz;
  sz = obj.m_data.size();

  if(sz > m_data.size())
    {
      unsigned int oldSz(m_data.size());

      m_data.resize(sz, NULL);
      for(unsigned int i = oldSz; i < sz; ++i)
        {
          m_data[i] = FASTUIDRAWnew std::vector<AntiAliasedTriangle>();
        }
    }

  for(unsigned int i = 0; i < sz; ++i)
    {
      assert(i < m_data.size() && m_data[i] != NULL);
      m_data[i]->reserve(m_data[i]->size() + obj.m_data[i]->size());
      m_data[i]->insert(m_data[i]->end(),
                        obj.m_data[i]->begin(),
                        obj.m_data[i]->end());
    }
}

////////////////////////////////
// EdgeData methods
void
EdgeData::
add_winding(uint64_t area, int w, unsigned int v)
{
  per_entry p;

  p.m_area = area;
  p.m_winding = w;
  p.m_vertex = v;
  m_entries.push_back(p);
}

int
EdgeData::
opposite_winding(int w, unsigned int v)
{
  if(m_entries.size() < 2)
    {
      /*
        edge with only one triangle, this edge is then
        made either from
          a) an external edge to the entire path (i.e. winding 0)
          b) a created edge from SubPath localization.
        In either case, we will return the same winding value w,
        to indicate that it is an internal edge.
      */
      return w;
    }

  if(m_entries.size() == 2)
    {
      return m_entries[0].canidate(w, v) ?
        m_entries[0].m_winding :
        m_entries[1].m_winding;
    }

  if(!m_sorted)
    {
      m_sorted = true;
      std::sort(m_entries.begin(), m_entries.end());
    }

  /* The entries are sorted in descending order by area,
     so the best canindate is then the first vertex that
     is different.
   */
  for(unsigned int i = 0, endi = m_entries.size(); i < endi; ++i)
    {
      if(m_entries[i].m_vertex != v)
        {
          return m_entries[i].m_winding;
        }
    }
  /* all vertices are same, this can happen from the discretization
     of the data. We punt and assume an internal edge.
   */
  return w;
}

////////////////////////////////
// OppositeEdgeTracker methods
void
OppositeEdgeTracker::
add_triangle(int w, uint64_t area,
             unsigned int v0, unsigned int v1, unsigned int v2)
{
  m_data[Edge(v0, v1)].add_winding(area, w, v2);
  m_data[Edge(v0, v2)].add_winding(area, w, v1);
  m_data[Edge(v1, v2)].add_winding(area, w, v0);
}

void
OppositeEdgeTracker::
add_aa_triangle(std::vector<AntiAliasedTriangle> &dst,
                int winding,
                const fastuidraw::vecN<unsigned int, 3> &tri)
{
  /* for each edge of T, get the winding number of the
     triangle that shares the edge with T.
   */
  AntiAliasedTriangle aa_tri;
  aa_tri[0].m_position = m_points[tri[0]];
  assert(m_data.find(Edge(tri[1], tri[2])) != m_data.end());
  aa_tri[0].m_winding_opposite = m_data[Edge(tri[1], tri[2])].opposite_winding(winding, tri[0]);

  aa_tri[1].m_position = m_points[tri[1]];
  assert(m_data.find(Edge(tri[0], tri[2])) != m_data.end());
  aa_tri[1].m_winding_opposite = m_data[Edge(tri[0], tri[2])].opposite_winding(winding, tri[1]);

  aa_tri[2].m_position = m_points[tri[2]];
  assert(m_data.find(Edge(tri[0], tri[1])) != m_data.end());
  aa_tri[2].m_winding_opposite = m_data[Edge(tri[0], tri[1])].opposite_winding(winding, tri[2]);

  dst.push_back(aa_tri);
}

void
OppositeEdgeTracker::
fill_aa_triangles(AntiAliasedTrianglesHoard &a,
                  const winding_index_hoard &raw_tris)
{
  for(winding_index_hoard::const_iterator iter = raw_tris.begin(),
        end = raw_tris.end(); iter != end; ++iter)
    {
      int winding_number(iter->first);
      fastuidraw::reference_counted_ptr<per_winding_data> tris(iter->second);
      if(tris)
        {
          std::vector<AntiAliasedTriangle> &dst(a.anti_aliased_triangles_writeable(winding_number));
          /* the index data for the triangle's is indices into m_points.
           */
          for(per_winding_data::index_list::const_iterator i = tris->indices().begin(),
                e = tris->indices().end(); i != e; /*nothing to increment */)
            {
              fastuidraw::vecN<unsigned int, 3> T;
              unsigned int k;

              for(k = 0; k < 3 && i != e; ++k, ++i)
                {
                  T[k] = *i;
                }
              assert(k == 3);
              add_aa_triangle(dst, winding_number, T);
            }
        }
    }

  for(std::map<Edge, EdgeData>::const_iterator iter = m_data.begin(),
        end = m_data.end(); iter != end; ++iter)
    {
      const Edge &edge(iter->first);
      const EdgeData &data(iter->second);
      if(data.m_entries.size() > 2)
        {
          std::cout << "Too many points on Edge [" << m_points[edge[0]]
                    << m_points.ipt(edge[0]) << ", "
                    << m_points[edge[1]]
                    << m_points.ipt(edge[1]) << "]\n";
          for(unsigned int i = 0, endi = data.m_entries.size(); i < endi; ++i)
            {
              const EdgeData::per_entry &p(data.m_entries[i]);
              std::cout << "\t" << m_points[p.m_vertex]
                        << m_points.ipt(p.m_vertex)
                        << ":" << "area = " << p.m_area
                        << ", w = " << p.m_winding
                        << "\n";
            }
        }
    }
}

/////////////////////////////////
// SubsetPrivate methods
SubsetPrivate::
SubsetPrivate(SubPath *Q, int max_recursion,
              std::vector<SubsetPrivate*> &out_values):
  m_ID(out_values.size()),
  m_bounds(Q->bounds()),
  m_painter_data(NULL),
  m_sizes_ready(false),
  m_sub_path(Q),
  m_children(NULL, NULL)
{
  out_values.push_back(this);
  if(max_recursion > 0 && m_sub_path->total_points() > SubsetConstants::points_per_subset)
    {
      fastuidraw::vecN<SubPath*, 2> C;

      C = Q->split();
      if(C[0]->total_points() < m_sub_path->total_points() || C[1]->total_points() < m_sub_path->total_points())
        {
          m_children[0] = FASTUIDRAWnew SubsetPrivate(C[0], max_recursion - 1, out_values);
          m_children[1] = FASTUIDRAWnew SubsetPrivate(C[1], max_recursion - 1, out_values);
          FASTUIDRAWdelete(m_sub_path);
          m_sub_path = NULL;
        }
      else
        {
          FASTUIDRAWdelete(C[0]);
          FASTUIDRAWdelete(C[1]);
        }
    }
}

SubsetPrivate::
~SubsetPrivate(void)
{
  if(m_sub_path != NULL)
    {
      assert(m_painter_data == NULL);
      assert(m_children[0] == NULL);
      assert(m_children[1] == NULL);
      FASTUIDRAWdelete(m_sub_path);
    }

  if(m_painter_data != NULL)
    {
      assert(m_sub_path == NULL);
      FASTUIDRAWdelete(m_painter_data);
    }

  if(m_children[0] != NULL)
    {
      assert(m_sub_path == NULL);
      assert(m_children[1] != NULL);
      FASTUIDRAWdelete(m_children[0]);
      FASTUIDRAWdelete(m_children[1]);
    }
}

unsigned int
SubsetPrivate::
select_subsets(ScratchSpacePrivate &scratch,
               bool with_anti_aliasing,
               fastuidraw::const_c_array<fastuidraw::vec3> clip_equations,
               const fastuidraw::float3x3 &clip_matrix_local,
               unsigned int max_attribute_cnt,
               unsigned int max_index_cnt,
               fastuidraw::c_array<unsigned int> dst)
{
  unsigned int return_value(0u);

  scratch.m_adjusted_clip_eqs.resize(clip_equations.size());
  for(unsigned int i = 0; i < clip_equations.size(); ++i)
    {
      /* transform clip equations from clip coordinates to
         local coordinates.
       */
      scratch.m_adjusted_clip_eqs[i] = clip_equations[i] * clip_matrix_local;
    }

  select_subsets_implement(scratch, with_anti_aliasing, dst, max_attribute_cnt, max_index_cnt, return_value);
  return return_value;
}

void
SubsetPrivate::
select_subsets_implement(ScratchSpacePrivate &scratch,
                         bool with_anti_aliasing,
                         fastuidraw::c_array<unsigned int> dst,
                         unsigned int max_attribute_cnt,
                         unsigned int max_index_cnt,
                         unsigned int &current)
{
  using namespace fastuidraw;
  using namespace fastuidraw::detail;

  vecN<vec2, 4> bb;
  bool unclipped;

  m_bounds.inflated_polygon(bb, 0.0f);
  unclipped = clip_against_planes(make_c_array(scratch.m_adjusted_clip_eqs),
                                  bb, scratch.m_clipped_rect,
                                  scratch.m_clip_scratch_floats,
                                  scratch.m_clip_scratch_vec2s);

  //completely clipped
  if(scratch.m_clipped_rect.empty())
    {
      return;
    }

  //completely unclipped or no children
  assert((m_children[0] == NULL) == (m_children[1] == NULL));
  if(unclipped || m_children[0] == NULL)
    {
      select_subsets_all_unculled(dst, with_anti_aliasing, max_attribute_cnt, max_index_cnt, current);
      return;
    }

  m_children[0]->select_subsets_implement(scratch, with_anti_aliasing, dst, max_attribute_cnt, max_index_cnt, current);
  m_children[1]->select_subsets_implement(scratch, with_anti_aliasing, dst, max_attribute_cnt, max_index_cnt, current);
}

void
SubsetPrivate::
select_subsets_all_unculled(fastuidraw::c_array<unsigned int> dst,
                            bool with_anti_aliasing,
                            unsigned int max_attribute_cnt,
                            unsigned int max_index_cnt,
                            unsigned int &current)
{
  if(!m_sizes_ready && m_children[0] == NULL)
    {
      /* we are going to need the attributes because
         the element will be selected.
       */
      make_ready_from_sub_path();
      assert(m_painter_data != NULL);
    }

  if(m_sizes_ready && can_fit_into_buffer(with_anti_aliasing, max_attribute_cnt, max_index_cnt))
    {
      dst[current] = m_ID;
      ++current;
    }
  else if(m_children[0] != NULL)
    {
      m_children[0]->select_subsets_all_unculled(dst, with_anti_aliasing, max_attribute_cnt, max_index_cnt, current);
      m_children[1]->select_subsets_all_unculled(dst, with_anti_aliasing, max_attribute_cnt, max_index_cnt, current);
      if(!m_sizes_ready)
        {
          m_sizes_ready = true;
          assert(m_children[0]->m_sizes_ready);
          assert(m_children[1]->m_sizes_ready);
          m_num_attributes = m_children[0]->m_num_attributes + m_children[1]->m_num_attributes;
          m_largest_index_block = m_children[0]->m_largest_index_block + m_children[1]->m_largest_index_block;
          m_num_aa_triangle_points = m_children[0]->m_num_aa_triangle_points + m_children[1]->m_num_aa_triangle_points;
        }
    }
  else
    {
      assert(m_sizes_ready);
      assert(!"Childless FilledPath::Subset has too many attributes or indices");
    }
}

void
SubsetPrivate::
make_ready(void)
{
  if(m_painter_data == NULL)
    {
      if(m_sub_path != NULL)
        {
          make_ready_from_sub_path();
        }
      else
        {
          make_ready_from_children();
        }
    }
}

void
SubsetPrivate::
make_ready_from_children(void)
{
  assert(m_children[0] != NULL);
  assert(m_children[1] != NULL);
  assert(m_sub_path == NULL);
  assert(m_painter_data == NULL);

  m_children[0]->make_ready();
  m_children[1]->make_ready();

  AttributeDataMerger merger(m_children[0]->painter_data(),
                             m_children[1]->painter_data());
  std::set<int> wnd;

  m_painter_data = FASTUIDRAWnew fastuidraw::PainterAttributeData();
  m_painter_data->set_data(merger);

  std::copy(m_children[0]->winding_numbers().begin(),
            m_children[0]->winding_numbers().end(),
            std::inserter(wnd, wnd.begin()));
  std::copy(m_children[1]->winding_numbers().begin(),
            m_children[1]->winding_numbers().end(),
            std::inserter(wnd, wnd.begin()));
  m_winding_numbers.resize(wnd.size());
  std::copy(wnd.begin(), wnd.end(), m_winding_numbers.begin());

  if(!m_sizes_ready)
    {
      m_sizes_ready = true;
      assert(m_children[0]->m_sizes_ready);
      assert(m_children[1]->m_sizes_ready);
      m_num_attributes = m_children[0]->m_num_attributes + m_children[1]->m_num_attributes;
      m_largest_index_block = m_children[0]->m_largest_index_block + m_children[1]->m_largest_index_block;
      m_num_aa_triangle_points = m_children[0]->m_num_aa_triangle_points + m_children[1]->m_num_aa_triangle_points;
    }
  m_aa_triangles.absorb(m_children[0]->m_aa_triangles);
  m_aa_triangles.absorb(m_children[1]->m_aa_triangles);
}

void
SubsetPrivate::
make_ready_from_sub_path(void)
{
  assert(m_children[0] == NULL);
  assert(m_children[1] == NULL);
  assert(m_sub_path != NULL);
  assert(m_painter_data == NULL);
  assert(!m_sizes_ready);

  AttributeDataFiller filler;
  builder B(*m_sub_path, filler.m_points, m_aa_triangles);
  unsigned int even_non_zero_start, zero_start;
  unsigned int m1, m2;

  B.fill_indices(filler.m_indices, filler.m_per_fill,
                 even_non_zero_start, zero_start);
  m_num_aa_triangle_points = 3 * m_aa_triangles.total_number_tris();

  fastuidraw::const_c_array<unsigned int> indices_ptr;
  indices_ptr = fastuidraw::make_c_array(filler.m_indices);
  filler.m_nonzero_winding_indices = indices_ptr.sub_array(0, zero_start);
  filler.m_odd_winding_indices = indices_ptr.sub_array(0, even_non_zero_start);
  filler.m_even_winding_indices = indices_ptr.sub_array(even_non_zero_start);
  filler.m_zero_winding_indices = indices_ptr.sub_array(zero_start);

  m_sizes_ready = true;
  m1 = fastuidraw::t_max(filler.m_nonzero_winding_indices.size(),
                         filler.m_zero_winding_indices.size());
  m2 = fastuidraw::t_max(filler.m_odd_winding_indices.size(),
                         filler.m_even_winding_indices.size());
  m_largest_index_block = fastuidraw::t_max(m1, m2);
  m_num_attributes = filler.m_points.size();

  m_winding_numbers.reserve(filler.m_per_fill.size());
  for(std::map<int, fastuidraw::const_c_array<unsigned int> >::iterator
        iter = filler.m_per_fill.begin(), end = filler.m_per_fill.end();
      iter != end; ++iter)
    {
      assert(!iter->second.empty());
      m_winding_numbers.push_back(iter->first);
    }

  /* now fill m_painter_data.
   */
  m_painter_data = FASTUIDRAWnew fastuidraw::PainterAttributeData();
  m_painter_data->set_data(filler);

  FASTUIDRAWdelete(m_sub_path);
  m_sub_path = NULL;

  #ifdef FASTUIDRAW_DEBUG
    {
      if(B.triangulation_failed())
        {
          /* On debug builds, print a warning.
           */
          std::cerr << "[" << __FILE__ << ", " << __LINE__
                    << "] Triangulation failed on tessellated path "
                    << this << "\n";
        }
    }
  #endif

}

/////////////////////////////////
// FilledPathPrivate methods
FilledPathPrivate::
FilledPathPrivate(const fastuidraw::TessellatedPath &P)
{
  SubPath *q;
  q = FASTUIDRAWnew SubPath(P);
  m_root = FASTUIDRAWnew SubsetPrivate(q, SubsetConstants::recursion_depth, m_subsets);
}

FilledPathPrivate::
~FilledPathPrivate()
{
  FASTUIDRAWdelete(m_root);
}

///////////////////////////////
//fastuidraw::FilledPath::ScratchSpace methods
fastuidraw::FilledPath::ScratchSpace::
ScratchSpace(void)
{
  m_d = FASTUIDRAWnew ScratchSpacePrivate();
}

fastuidraw::FilledPath::ScratchSpace::
~ScratchSpace(void)
{
  ScratchSpacePrivate *d;
  d = static_cast<ScratchSpacePrivate*>(m_d);
  FASTUIDRAWdelete(d);
  m_d = NULL;
}

//////////////////////////////////////////////
// fastuidraw::FilledPath::DataWriter methods
fastuidraw::FilledPath::DataWriter::
DataWriter(void)
{
  m_d = FASTUIDRAWnew DataWriterPrivate();
}

fastuidraw::FilledPath::DataWriter::
DataWriter(const DataWriter &obj)
{
  DataWriterPrivate *obj_d;
  obj_d = static_cast<DataWriterPrivate*>(obj.m_d);
  m_d = FASTUIDRAWnew DataWriterPrivate(*obj_d);
}

fastuidraw::FilledPath::DataWriter::
~DataWriter()
{
  DataWriterPrivate *d;
  d = static_cast<DataWriterPrivate*>(m_d);
  FASTUIDRAWdelete(d);
  m_d = NULL;
}

void
fastuidraw::FilledPath::DataWriter::
swap(DataWriter &obj)
{
  std::swap(obj.m_d, m_d);
}

const fastuidraw::FilledPath::DataWriter&
fastuidraw::FilledPath::DataWriter::
operator=(const DataWriter &rhs)
{
  if(&rhs != this)
    {
      DataWriter tmp(rhs);
      swap(tmp);
    }
  return *this;
}

unsigned int
fastuidraw::FilledPath::DataWriter::
number_attribute_chunks(void) const
{
  DataWriterPrivate *d;
  d = reinterpret_cast<DataWriterPrivate*>(m_d);
  return d->m_attribute_chunks.size();
}

unsigned int
fastuidraw::FilledPath::DataWriter::
number_attributes(unsigned int attribute_chunk) const
{
  DataWriterPrivate *d;
  d = reinterpret_cast<DataWriterPrivate*>(m_d);
  assert(attribute_chunk < d->m_attribute_chunks.size());
  return d->m_attribute_chunks[attribute_chunk].m_attribs.size();
}

unsigned int
fastuidraw::FilledPath::DataWriter::
number_index_chunks(void) const
{
  DataWriterPrivate *d;
  d = reinterpret_cast<DataWriterPrivate*>(m_d);
  return d->m_index_chunks.size();
}

unsigned int
fastuidraw::FilledPath::DataWriter::
number_indices(unsigned int index_chunk) const
{
  DataWriterPrivate *d;
  d = reinterpret_cast<DataWriterPrivate*>(m_d);
  assert(index_chunk < d->m_index_chunks.size());
  return d->m_index_chunks[index_chunk].m_indices.size();
}

unsigned int
fastuidraw::FilledPath::DataWriter::
attribute_chunk_selection(unsigned int index_chunk) const
{
  DataWriterPrivate *d;
  d = reinterpret_cast<DataWriterPrivate*>(m_d);
  assert(index_chunk < d->m_index_chunks.size());
  return d->m_index_chunks[index_chunk].m_attrib_chunk;
}

void
fastuidraw::FilledPath::DataWriter::
write_indices(c_array<PainterIndex> dst,
              unsigned int index_offset_value,
              unsigned int index_chunk) const
{
  const_c_array<PainterIndex> src;
  DataWriterPrivate *d;
  d = reinterpret_cast<DataWriterPrivate*>(m_d);

  assert(index_chunk < d->m_index_chunks.size());
  src = d->m_index_chunks[index_chunk].m_indices;

  assert(dst.size() == src.size());
  for(unsigned int i = 0; i < dst.size(); ++i)
    {
      dst[i] = src[i] + index_offset_value;
    }
}

void
fastuidraw::FilledPath::DataWriter::
write_attributes(c_array<PainterAttribute> dst,
                 unsigned int attribute_chunk) const
{
  const_c_array<PainterAttribute> src;
  DataWriterPrivate *d;

  d = reinterpret_cast<DataWriterPrivate*>(m_d);

  assert(attribute_chunk < d->m_attribute_chunks.size());
  src = d->m_attribute_chunks[attribute_chunk].m_attribs;

  assert(dst.size() == src.size());
  /* TODO: if doing anti-aliasing apply bit-pattern
     logic to attributes.
   */
  std::copy(src.begin(), src.end(), dst.begin());
}

/////////////////////////////////
// fastuidraw::FilledPath::Subset methods
fastuidraw::FilledPath::Subset::
Subset(void *d):
  m_d(d)
{
}

fastuidraw::const_c_array<fastuidraw::FilledPath::TriangleWithOppositeEdgeData>
fastuidraw::FilledPath::Subset::
triangles_with_opposite_edge_data(int winding) const
{
  SubsetPrivate *d;
  d = static_cast<SubsetPrivate*>(m_d);
  return d->aa_triangles().anti_aliased_triangles(winding);
}

const fastuidraw::PainterAttributeData&
fastuidraw::FilledPath::Subset::
painter_data(void) const
{
  SubsetPrivate *d;
  d = static_cast<SubsetPrivate*>(m_d);
  return d->painter_data();
}

fastuidraw::const_c_array<int>
fastuidraw::FilledPath::Subset::
winding_numbers(void) const
{
  SubsetPrivate *d;
  d = static_cast<SubsetPrivate*>(m_d);
  return d->winding_numbers();
}

unsigned int
fastuidraw::FilledPath::Subset::
chunk_from_winding_number(int winding_number)
{
  /* basic idea:
     - start counting at fill_rule_data_count
     - ordering is: 1, -1, 2, -2, ...
  */
  int value, sg;

  if(winding_number == 0)
    {
      return fastuidraw::PainterEnums::complement_nonzero_fill_rule;
    }

  value = std::abs(winding_number);
  sg = (winding_number < 0) ? 1 : 0;
  return fastuidraw::PainterEnums::fill_rule_data_count + sg + 2 * (value - 1);
}

unsigned int
fastuidraw::FilledPath::Subset::
chunk_from_fill_rule(enum PainterEnums::fill_rule_t fill_rule)
{
  assert(fill_rule < fastuidraw::PainterEnums::fill_rule_data_count);
  return fill_rule;
}

///////////////////////////////////////
// fastuidraw::FilledPath methods
fastuidraw::FilledPath::
FilledPath(const TessellatedPath &P)
{
  m_d = FASTUIDRAWnew FilledPathPrivate(P);
}

fastuidraw::FilledPath::
~FilledPath()
{
  FilledPathPrivate *d;
  d = static_cast<FilledPathPrivate*>(m_d);
  FASTUIDRAWdelete(d);
  m_d = NULL;
}

unsigned int
fastuidraw::FilledPath::
number_subsets(void) const
{
  FilledPathPrivate *d;
  d = static_cast<FilledPathPrivate*>(m_d);
  return d->m_subsets.size();
}


fastuidraw::FilledPath::Subset
fastuidraw::FilledPath::
subset(unsigned int I) const
{
  FilledPathPrivate *d;
  SubsetPrivate *p;

  d = static_cast<FilledPathPrivate*>(m_d);
  assert(I < d->m_subsets.size());
  p = d->m_subsets[I];
  p->make_ready();

  return Subset(p);
}

unsigned int
fastuidraw::FilledPath::
select_subsets(ScratchSpace &work_room,
               bool triangle_with_opposite_data,
               const_c_array<vec3> clip_equations,
               const float3x3 &clip_matrix_local,
               unsigned int max_attribute_cnt,
               unsigned int max_index_cnt,
               c_array<unsigned int> dst) const
{
  FilledPathPrivate *d;
  unsigned int return_value;

  d = static_cast<FilledPathPrivate*>(m_d);
  assert(dst.size() >= d->m_subsets.size());
  /* TODO:
       - have another method in SubsetPrivate called
         "fast_select_subsets" which ignores the requirements
         coming from max_attribute_cnt and max_index_cnt.
         By ignoring this requirement, we do NOT need
         to do call make_ready() for any SubsetPrivate
         object chosen.
       - have the fast_select_subsets() also return
         if paths needed require triangulation.
       - if there such, spawn a thread and let the
         caller decide if to wait for the thread to
         finish before proceeding or to do something
         else (like use a lower level of detail that
         is ready). Another alternatic is to return
         what Subset's need to have triangulation done
         and spawn a set of threads to do the job (!)
       - All this work means we need to make SubsetPrivate
         thread safe (with regards to the SubsetPrivate
         being made ready via make_ready()).
   */
  return_value= d->m_root->select_subsets(*static_cast<ScratchSpacePrivate*>(work_room.m_d),
                                          triangle_with_opposite_data,
                                          clip_equations, clip_matrix_local,
                                          max_attribute_cnt, max_index_cnt, dst);

  return return_value;
}

void
fastuidraw::FilledPath::
compute_writer(ScratchSpace &scratch_space,
               bool with_anti_aliasing,
               const CustomFillRuleBase &fill_rule,
               const_c_array<vec3> clip_equations,
               const float3x3 &clip_matrix_local,
               unsigned int max_attribute_cnt,
               unsigned int max_index_cnt,
               DataWriter &dst) const
{
  DataWriterPrivate *dst_d;
  unsigned int num;

  dst_d = reinterpret_cast<DataWriterPrivate*>(dst.m_d);

  dst_d->m_attribute_chunks.clear();
  dst_d->m_index_chunks.clear();
  dst_d->m_with_anti_aliasing = with_anti_aliasing;

  dst_d->m_subset_selector.resize(number_subsets());
  num = select_subsets(scratch_space, with_anti_aliasing,
                       clip_equations, clip_matrix_local,
                       max_attribute_cnt, max_index_cnt,
                       make_c_array(dst_d->m_subset_selector));

  if(num == 0)
    {
      return;
    }

  dst_d->m_attribute_chunks.reserve(num);
  dst_d->m_index_chunks.reserve(num);
  for(unsigned int i = 0; i < num; ++i)
    {
      Subset S(subset(dst_d->m_subset_selector[i]));
      SubsetPrivate *sd;
      const_c_array<int> windings;
      const unsigned int ATTRIB_CHUNK_NOT_TAKEN = ~0u;
      unsigned int attrib_chunk;

      sd = reinterpret_cast<SubsetPrivate*>(S.m_d);
      windings = sd->winding_numbers();
      attrib_chunk = ATTRIB_CHUNK_NOT_TAKEN;

      for(unsigned int i = 0; i < windings.size(); ++i)
        {
          int w;

          w = windings[i];
          if(fill_rule(w))
            {
              unsigned int index_chunk;
              const_c_array<PainterIndex> indices;

              if(attrib_chunk == ATTRIB_CHUNK_NOT_TAKEN)
                {
                  attrib_chunk = dst_d->m_attribute_chunks.size();
                  dst_d->m_attribute_chunks.push_back(DataWriterPrivate::per_attrib_chunk(sd));
                }

              index_chunk = Subset::chunk_from_winding_number(w);
              indices = sd->painter_data().index_data_chunk(index_chunk);

              dst_d->m_index_chunks.push_back(DataWriterPrivate::per_index_chunk(indices, attrib_chunk));
            }
        }
    }
}

void
fastuidraw::FilledPath::
compute_writer(ScratchSpace &scratch_space,
               bool with_anti_aliasing,
               enum PainterEnums::fill_rule_t fill_rule,
               const_c_array<vec3> clip_equations,
               const float3x3 &clip_matrix_local,
               unsigned int max_attribute_cnt,
               unsigned int max_index_cnt,
               DataWriter &dst) const
{
  compute_writer(scratch_space, with_anti_aliasing,
                 CustomFillRuleFunction(fill_rule),
                 clip_equations, clip_matrix_local,
                 max_attribute_cnt, max_index_cnt, dst);
}
