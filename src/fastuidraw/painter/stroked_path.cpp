/*!
 * \file stroked_path.cpp
 * \brief file stroked_path.cpp
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
#include <complex>
#include <algorithm>

#include <fastuidraw/tessellated_path.hpp>
#include <fastuidraw/path.hpp>
#include <fastuidraw/painter/stroked_path.hpp>
#include <fastuidraw/painter/stroked_caps_joins.hpp>
#include <fastuidraw/painter/painter_attribute_data.hpp>
#include <fastuidraw/painter/painter_attribute_data_filler.hpp>
#include <fastuidraw/painter/painter_dashed_stroke_shader_set.hpp>
#include "../private/util_private.hpp"
#include "../private/util_private_ostream.hpp"
#include "../private/bounding_box.hpp"
#include "../private/path_util_private.hpp"
#include "../private/clip.hpp"

namespace
{
  inline
  uint32_t
  pack_data(int on_boundary,
            enum fastuidraw::StrokedPoint::offset_type_t pt,
            uint32_t depth)
  {
    using namespace fastuidraw;
    FASTUIDRAWassert(on_boundary == 0 || on_boundary == 1);

    uint32_t bb(on_boundary), pp(pt);
    return pack_bits(StrokedPoint::offset_type_bit0,
                     StrokedPoint::offset_type_num_bits, pp)
      | pack_bits(StrokedPoint::boundary_bit, 1u, bb)
      | pack_bits(StrokedPoint::depth_bit0,
                  StrokedPoint::depth_num_bits, depth);
  }

  template<typename T>
  class OrderingEntry:public T
  {
  public:
    OrderingEntry(const T &src, unsigned int chunk):
      T(src),
      m_chunk(chunk),
      m_depth(0)
    {}

    unsigned int m_chunk;
    unsigned int m_depth;
  };

  class SingleSubEdge
  {
  public:
    SingleSubEdge()
    {}

    explicit
    SingleSubEdge(const fastuidraw::TessellatedPath::segment &seg,
                  bool is_closing_edge);

    void
    split_sub_edge(int splitting_coordinate,
                   fastuidraw::vecN<SingleSubEdge, 2> &out_edges,
                   float split_value) const;

    fastuidraw::vec2 m_pt0, m_pt1;
    float m_distance_from_edge_start;
    float m_distance_from_contour_start;
    float m_edge_length;
    float m_open_contour_length;
    float m_closed_contour_length;

    bool m_of_closing_edge;
    fastuidraw::vec2 m_normal, m_delta;
    float m_sub_edge_length;

    bool m_has_bevel;
    float m_bevel_lambda;
    fastuidraw::vec2 m_bevel_normal;
  };

  /* A SubEdgeCullingHierarchy represents a hierarchy choice of
   *  what sub-edges land in each element of a hierarchy.
   */
  class SubEdgeCullingHierarchy:fastuidraw::noncopyable
  {
  public:
    static
    SubEdgeCullingHierarchy*
    create(const fastuidraw::TessellatedPath &P);

    ~SubEdgeCullingHierarchy();

    bool
    has_children(void) const
    {
      bool b0(m_children[0] != nullptr);
      bool b1(m_children[1] != nullptr);

      FASTUIDRAWassert(b0 == b1);
      FASTUIDRAWunused(b0);
      FASTUIDRAWunused(b1);
      return m_children[0] != nullptr;
    }

    const SubEdgeCullingHierarchy*
    child(unsigned int i) const
    {
      return m_children[i];
    }

    fastuidraw::c_array<const SingleSubEdge>
    non_closing_edges(void) const
    {
      return m_sub_edges.m_non_closing;
    }

    fastuidraw::c_array<const SingleSubEdge>
    closing_edges(void) const
    {
      return m_sub_edges.m_closing;
    }

    const fastuidraw::BoundingBox<float>&
    bounding_box(void) const
    {
      return m_bb;
    }

  private:

    template<typename T>
    class PartitionedData
    {
    public:
      void
      init(std::vector<T> &data, unsigned int num_non_closing)
      {
        fastuidraw::c_array<const T> all_data;

        m_data.swap(data);
        all_data = fastuidraw::make_c_array(m_data);

        m_non_closing = all_data.sub_array(0, num_non_closing);
        m_closing = all_data.sub_array(num_non_closing);
      }

      /* all data, with data so that data with closing
       *  is only in the back
       */
      std::vector<T> m_data;

      /* array into m_data of data of non-closing edges
       */
      fastuidraw::c_array<const T> m_non_closing;

      /* array into m_data of data of ONLY closing edges
       */
      fastuidraw::c_array<const T> m_closing;
    };

    enum
      {
        splitting_threshhold = 20
      };

    SubEdgeCullingHierarchy(const fastuidraw::BoundingBox<float> &start_box,
                            std::vector<SingleSubEdge> &data, unsigned int num_non_closing_edges);

    /* a value of -1 means to NOT split.
     */
    static
    int
    choose_splitting_coordinate(const fastuidraw::BoundingBox<float> &start_box,
                                fastuidraw::c_array<const SingleSubEdge> data,
                                float &split_value);

    static
    void
    create_lists(const fastuidraw::TessellatedPath &P,
                 std::vector<SingleSubEdge> &data, unsigned int &num_non_closing_edges,
                 fastuidraw::BoundingBox<float> &bx);

    static
    void
    process_edge(const fastuidraw::TessellatedPath &P,
                 unsigned int contour, unsigned int edge,
                 std::vector<SingleSubEdge> &dst,
                 fastuidraw::BoundingBox<float> &bx);

    template<typename T>
    static
    void
    check_closing_at_end(const std::vector<T> &data, unsigned int num_non_closing);

    static
    float
    compute_lambda(const fastuidraw::vec2 &n0, const fastuidraw::vec2 &n1)
    {
      fastuidraw::vec2 v1(n1.y(), -n1.x());
      float d;

      d = fastuidraw::dot(v1, n0);
      return (d > 0.0f) ? -1.0f : 1.0f;
    }

    /* children, if any
     */
    fastuidraw::vecN<SubEdgeCullingHierarchy*, 2> m_children;

    /* what edges of this, only non-empty if has no children.
     */
    PartitionedData<SingleSubEdge> m_sub_edges;

    /* bounding box of this
     */
    fastuidraw::BoundingBox<float> m_bb;
  };

  class ScratchSpacePrivate:fastuidraw::noncopyable
  {
  public:
    std::vector<fastuidraw::vec3> m_adjusted_clip_eqs;
    std::vector<fastuidraw::vec2> m_clipped_rect;

    fastuidraw::vecN<std::vector<fastuidraw::vec2>, 2> m_clip_scratch_vec2s;
    std::vector<float> m_clip_scratch_floats;
  };

  class EdgeRanges
  {
  public:
    /* range where vertices and indices of edges are located
     */
    fastuidraw::range_type<unsigned int> m_vertex_data_range;
    fastuidraw::range_type<unsigned int> m_index_data_range;

    /* range of depth values applied to edges.
     */
    fastuidraw::range_type<unsigned int> m_depth_range;

    /* chunk in PainterAttributeData holding the range
     *  of indices and vertices above.
     */
    unsigned int m_chunk;

    /* source of data, only non-empty if EdgeAttributeFiller
     *  should create attribute/index data at the ranges
     *  above.
     */
    fastuidraw::c_array<const SingleSubEdge> m_src;

    bool
    chunk_fits(unsigned int max_attribute_cnt,
               unsigned int max_index_cnt) const
    {
      return m_vertex_data_range.difference() <= max_attribute_cnt
        && m_index_data_range.difference() <= max_index_cnt;
    }

    bool
    non_empty(void) const
    {
      return m_vertex_data_range.m_end > m_vertex_data_range.m_begin;
    }
  };

  class ChunkSetPrivate:fastuidraw::noncopyable
  {
  public:
    void
    reset(void)
    {
      m_edge_chunks.clear();
    }

    void
    add_edge_chunk(const EdgeRanges &ed);

    fastuidraw::c_array<const unsigned int>
    edge_chunks(void) const
    {
      return fastuidraw::make_c_array(m_edge_chunks);
    }

    fastuidraw::StrokedCapsJoins::ChunkSet m_caps_joins;

  private:
    std::vector<unsigned int> m_edge_chunks;
  };

  /* Subset of a StrokedPath. Edges are to be placed into
   *  the store as follows:
   *    1. child0 edges
   *    2. child1 edges
   *    3. edges (i.e. from SubEdgeCullingHierarchy::m_sub_edges)
   *
   *  with the invariant thats that
   */
  class StrokedPathSubset
  {
  public:
    enum
      {
        points_per_segment = 6,
        triangles_per_segment = points_per_segment - 2,
        indices_per_segment_without_bevel = 3 * triangles_per_segment,
      };

    class CreationValues
    {
    public:
      CreationValues(void):
        m_non_closing_edge_vertex_cnt(0),
        m_non_closing_edge_index_cnt(0),
        m_non_closing_edge_chunk_cnt(0),
        m_closing_edge_vertex_cnt(0),
        m_closing_edge_index_cnt(0),
        m_closing_edge_chunk_cnt(0)
      {}

      unsigned int m_non_closing_edge_vertex_cnt;
      unsigned int m_non_closing_edge_index_cnt;
      unsigned int m_non_closing_edge_chunk_cnt;

      unsigned int m_closing_edge_vertex_cnt;
      unsigned int m_closing_edge_index_cnt;
      unsigned int m_closing_edge_chunk_cnt;
    };

    static
    StrokedPathSubset*
    create(const SubEdgeCullingHierarchy *src,
           CreationValues &out_values);

    ~StrokedPathSubset();

    void
    compute_chunks(bool include_closing_edge,
                   ScratchSpacePrivate &work_room,
                   fastuidraw::c_array<const fastuidraw::vec3> clip_equations,
                   const fastuidraw::float3x3 &clip_matrix_local,
                   const fastuidraw::vec2 &recip_dimensions,
                   float pixels_additional_room,
                   float item_space_additional_room,
                   unsigned int max_attribute_cnt,
                   unsigned int max_index_cnt,
                   ChunkSetPrivate &dst);

    bool
    have_children(void) const
    {
      bool b0(m_children[0] != nullptr);
      bool b1(m_children[1] != nullptr);

      FASTUIDRAWassert(b0 == b1);
      FASTUIDRAWunused(b0);
      FASTUIDRAWunused(b1);
      return m_children[0] != nullptr;
    }

    const StrokedPathSubset*
    child(unsigned int I) const
    {
      FASTUIDRAWassert(have_children());
      FASTUIDRAWassert(I == 0 || I == 1);
      return m_children[I];
    }

    const EdgeRanges&
    non_closing_edges(void) const
    {
      return m_non_closing_edges;
    }

    const EdgeRanges&
    closing_edges(void) const
    {
      return m_closing_edges;
    }

  private:
    class PostProcessVariables
    {
    public:
      PostProcessVariables(void):
        m_edge_depth(0),
        m_closing_edge_depth(0)
      {}

      unsigned int m_edge_depth, m_closing_edge_depth;
    };

    StrokedPathSubset(CreationValues &out_values,
                      const SubEdgeCullingHierarchy *src);

    void
    compute_chunks_implement(bool include_closing_edge,
                             ScratchSpacePrivate &work_room,
                             float item_space_additional_room,
                             unsigned int max_attribute_cnt,
                             unsigned int max_index_cnt,
                             ChunkSetPrivate &dst);

    void
    compute_chunks_take_all(bool include_closing_edge,
                            unsigned int max_attribute_cnt,
                            unsigned int max_index_cnt,
                            ChunkSetPrivate &dst);

    static
    void
    increment_vertices_indices(fastuidraw::c_array<const SingleSubEdge> src,
                               unsigned int &vertex_cnt,
                               unsigned int &index_cnt);
    void
    post_process(PostProcessVariables &variables,
                 const CreationValues &constants);

    fastuidraw::vecN<StrokedPathSubset*, 2> m_children;

    /* book keeping for edges. */
    EdgeRanges m_non_closing_edges, m_closing_edges;

    fastuidraw::BoundingBox<float> m_bb;

    bool m_empty_subset;
  };

  class EdgeAttributeFiller:public fastuidraw::PainterAttributeDataFiller
  {
  public:
    explicit
    EdgeAttributeFiller(const StrokedPathSubset *src,
                        const fastuidraw::TessellatedPath &P,
                        const StrokedPathSubset::CreationValues &cnts);

    virtual
    void
    compute_sizes(unsigned int &num_attributes,
                  unsigned int &num_indices,
                  unsigned int &num_attribute_chunks,
                  unsigned int &num_index_chunks,
                  unsigned int &number_z_ranges) const;

    virtual
    void
    fill_data(fastuidraw::c_array<fastuidraw::PainterAttribute> attribute_data,
              fastuidraw::c_array<fastuidraw::PainterIndex> index_data,
              fastuidraw::c_array<fastuidraw::c_array<const fastuidraw::PainterAttribute> > attribute_chunks,
              fastuidraw::c_array<fastuidraw::c_array<const fastuidraw::PainterIndex> > index_chunks,
              fastuidraw::c_array<fastuidraw::range_type<int> > zranges,
              fastuidraw::c_array<int> index_adjusts) const;
  private:
    void
    fill_data_worker(const StrokedPathSubset *e,
                     fastuidraw::c_array<fastuidraw::PainterAttribute> attribute_data,
                     fastuidraw::c_array<fastuidraw::PainterIndex> index_data,
                     fastuidraw::c_array<fastuidraw::c_array<const fastuidraw::PainterAttribute> > attribute_chunks,
                     fastuidraw::c_array<fastuidraw::c_array<const fastuidraw::PainterIndex> > index_chunks,
                     fastuidraw::c_array<fastuidraw::range_type<int> > zranges,
                     fastuidraw::c_array<int> index_adjusts) const;

    void
    build_chunk(const EdgeRanges &edge,
                fastuidraw::c_array<fastuidraw::PainterAttribute> attribute_data,
                fastuidraw::c_array<fastuidraw::PainterIndex> index_data,
                fastuidraw::c_array<fastuidraw::c_array<const fastuidraw::PainterAttribute> > attribute_chunks,
                fastuidraw::c_array<fastuidraw::c_array<const fastuidraw::PainterIndex> > index_chunks,
                fastuidraw::c_array<fastuidraw::range_type<int> > zranges,
                fastuidraw::c_array<int> index_adjusts) const;

    void
    process_sub_edge(const SingleSubEdge &sub_edge, unsigned int depth,
                     fastuidraw::c_array<fastuidraw::PainterAttribute> attribute_data,
                     fastuidraw::c_array<fastuidraw::PainterIndex> index_data,
                     unsigned int &vertex_offset, unsigned int &index_offset) const;

    const StrokedPathSubset *m_src;
    const fastuidraw::TessellatedPath &m_P;
    unsigned int m_total_vertex_cnt, m_total_index_cnt;
    unsigned int m_total_number_chunks;
  };

  template<typename T>
  class PreparedAttributeData
  {
  public:
    PreparedAttributeData(void):
      m_ready(false)
    {}

    /* must be called before the first call to data().
     */
    void
    mark_as_empty(void)
    {
      m_ready = true;
    }

    const fastuidraw::PainterAttributeData&
    data(const StrokedPathSubset *st)
    {
      if (!m_ready)
        {
          m_data.set_data(T(st));
          m_ready = true;
        }
      return m_data;
    }

  private:
    fastuidraw::PainterAttributeData m_data;
    bool m_ready;
  };

  class StrokedPathPrivate:fastuidraw::noncopyable
  {
  public:
    explicit
    StrokedPathPrivate(const fastuidraw::TessellatedPath &P,
                       const fastuidraw::StrokedCapsJoins::Builder &b);
    ~StrokedPathPrivate();

    void
    create_edges(const fastuidraw::TessellatedPath &P);

    static
    void
    ready_builder(const fastuidraw::TessellatedPath *tess,
                  fastuidraw::StrokedCapsJoins::Builder &b);

    static
    void
    ready_builder_contour(const fastuidraw::TessellatedPath *tess,
                          unsigned int contour,
                          fastuidraw::StrokedCapsJoins::Builder &b);

    fastuidraw::StrokedCapsJoins m_caps_joins;
    StrokedPathSubset* m_subset;
    fastuidraw::PainterAttributeData m_edges;

    fastuidraw::vecN<unsigned int, 2> m_chunk_of_edges;

    bool m_empty_path;
    fastuidraw::PainterAttributeData m_empty_data;
  };

}

////////////////////////////////////////
// SingleSubEdge methods
SingleSubEdge::
SingleSubEdge(const fastuidraw::TessellatedPath::segment &seg,
              bool is_closing_edge):
  m_pt0(seg.m_p),
  m_pt1(seg.m_data),
  m_distance_from_edge_start(seg.m_distance_from_edge_start),
  m_distance_from_contour_start(seg.m_distance_from_contour_start),
  m_edge_length(seg.m_edge_length),
  m_open_contour_length(seg.m_open_contour_length),
  m_closed_contour_length(seg.m_closed_contour_length),
  m_of_closing_edge(is_closing_edge),
  m_delta(seg.m_data - seg.m_p),
  m_sub_edge_length(m_delta.magnitude())
{
  const float mag_tol = 0.000001f;

  FASTUIDRAWassert(seg.m_type == fastuidraw::TessellatedPath::line_segment);
  if (m_sub_edge_length >= mag_tol)
    {
      m_normal = fastuidraw::vec2(-m_delta.y(), m_delta.x()) / m_sub_edge_length;
    }
  else
    {
      m_normal = fastuidraw::vec2(1.0f, 0.0f);
    }
}

void
SingleSubEdge::
split_sub_edge(int splitting_coordinate,
               fastuidraw::vecN<SingleSubEdge, 2> &out_edges,
               float split_value) const
{
  float s, t, v0, v1;
  fastuidraw::vec2 p;

  v0 = m_pt0[splitting_coordinate];
  v1 = m_pt1[splitting_coordinate];
  t = (split_value - v0) / (v1 - v0);
  s = 1.0 - t;

  p = s * m_pt0 + t * m_pt1;

  out_edges[0].m_pt0 = m_pt0;
  out_edges[0].m_pt1 = p;
  out_edges[0].m_distance_from_edge_start = m_distance_from_edge_start;
  out_edges[0].m_distance_from_contour_start = m_distance_from_contour_start;
  out_edges[0].m_sub_edge_length = t * m_sub_edge_length;

  out_edges[1].m_pt0 = p;
  out_edges[1].m_pt1 = m_pt1;
  out_edges[1].m_distance_from_edge_start = m_distance_from_edge_start + out_edges[0].m_sub_edge_length;
  out_edges[1].m_distance_from_contour_start = m_distance_from_contour_start + out_edges[0].m_sub_edge_length;
  out_edges[1].m_sub_edge_length = s * m_sub_edge_length;

  for(unsigned int i = 0; i < 2; ++i)
    {
      out_edges[i].m_normal = m_normal;
      out_edges[i].m_delta = out_edges[i].m_pt1 - out_edges[i].m_pt0;
      out_edges[i].m_of_closing_edge = m_of_closing_edge;
      out_edges[i].m_has_bevel = m_has_bevel && (i == 0);
      out_edges[i].m_bevel_lambda = (i == 0) ? m_bevel_lambda : 0.0f;
      out_edges[i].m_bevel_normal = (i == 0) ? m_bevel_normal : fastuidraw::vec2(0.0f, 0.0f);

      out_edges[i].m_edge_length = m_edge_length;
      out_edges[i].m_open_contour_length = m_open_contour_length;
      out_edges[i].m_closed_contour_length = m_closed_contour_length;
    }
}

//////////////////////////////////////////////
// SubEdgeCullingHierarchy methods
SubEdgeCullingHierarchy*
SubEdgeCullingHierarchy::
create(const fastuidraw::TessellatedPath &P)
{
  std::vector<SingleSubEdge> data;
  fastuidraw::BoundingBox<float> bx;
  unsigned int num_non_closing_edges;
  SubEdgeCullingHierarchy *return_value;

  create_lists(P, data, num_non_closing_edges, bx);
  return_value =  FASTUIDRAWnew SubEdgeCullingHierarchy(bx, data, num_non_closing_edges);
  return return_value;
}

void
SubEdgeCullingHierarchy::
create_lists(const fastuidraw::TessellatedPath &P,
             std::vector<SingleSubEdge> &data, unsigned int &num_non_closing_edges,
             fastuidraw::BoundingBox<float> &bx)
{
  /* place data of closing sub-edges at the tail of the lists. */
  for(unsigned int o = 0; o < P.number_contours(); ++o)
    {
      for(unsigned int e = 0, ende = P.number_edges(o); e + 1 < ende; ++e)
        {
          process_edge(P, o, e, data, bx);
        }
    }

  num_non_closing_edges = data.size();
  for(unsigned int o = 0; o < P.number_contours(); ++o)
    {
      if (P.number_edges(o) > 0)
        {
          process_edge(P, o, P.number_edges(o) - 1, data, bx);
        }
    }
}

void
SubEdgeCullingHierarchy::
process_edge(const fastuidraw::TessellatedPath &P,
             unsigned int contour, unsigned int edge,
             std::vector<SingleSubEdge> &dst, fastuidraw::BoundingBox<float> &bx)
{
  fastuidraw::range_type<unsigned int> R;
  fastuidraw::c_array<const fastuidraw::TessellatedPath::segment> src_segments(P.segment_data());
  bool is_closing_edge;

  is_closing_edge = (edge + 1 == P.number_edges(contour));
  R = P.edge_range(contour, edge);
  FASTUIDRAWassert(R.m_end >= R.m_begin);

  for(unsigned int i = R.m_begin; i < R.m_end; ++i)
    {
      /* for the edge connecting src_pts[i] to src_pts[i+1]
       */
      SingleSubEdge sub_edge(src_segments[i], is_closing_edge);

      if (i == R.m_begin)
        {
          sub_edge.m_bevel_lambda = 0.0f;
          sub_edge.m_has_bevel = false;
          sub_edge.m_bevel_normal = fastuidraw::vec2(0.0f, 0.0f);
        }
      else
        {
          sub_edge.m_bevel_lambda = compute_lambda(dst.back().m_normal, sub_edge.m_normal);
          sub_edge.m_has_bevel = true;
          sub_edge.m_bevel_normal = dst.back().m_normal;
        }

      dst.push_back(sub_edge);
      bx.union_point(sub_edge.m_pt0);
      bx.union_point(sub_edge.m_pt1);
    }
}

template<typename T>
void
SubEdgeCullingHierarchy::
check_closing_at_end(const std::vector<T> &data, unsigned int num_non_closing)
{
  FASTUIDRAWassert(num_non_closing <= data.size());
  #ifdef FASTUIDRAW_DEBUG
    {
      for(unsigned int i = 0; i < num_non_closing; ++i)
        {
          FASTUIDRAWassert(!data[i].m_of_closing_edge);
        }
      for(unsigned int i = num_non_closing, endi = data.size(); i < endi; ++i)
        {
          FASTUIDRAWassert(data[i].m_of_closing_edge);
        }
    }
  #else
    {
      FASTUIDRAWunused(data);
      FASTUIDRAWunused(num_non_closing);
    }
  #endif
}

SubEdgeCullingHierarchy::
SubEdgeCullingHierarchy(const fastuidraw::BoundingBox<float> &start_box,
                        std::vector<SingleSubEdge> &edges, unsigned int num_non_closing_edges):
  m_children(nullptr, nullptr),
  m_bb(start_box)
{
  int c;
  float mid_point;

  FASTUIDRAWassert(!start_box.empty());
  check_closing_at_end(edges, num_non_closing_edges);

  c = choose_splitting_coordinate(start_box, fastuidraw::make_c_array(edges), mid_point);
  if (c != -1)
    {
      fastuidraw::vecN<fastuidraw::BoundingBox<float>, 2> child_boxes;
      fastuidraw::vecN<std::vector<SingleSubEdge>, 2> child_sub_edges;
      fastuidraw::vecN<unsigned int, 2> child_num_non_closing_edges(0, 0);
      fastuidraw::vecN<unsigned int, 2> child_num_non_closing_joins(0, 0);

      for(const SingleSubEdge &sub_edge : edges)
        {
          bool sA, sB;

          sA = (sub_edge.m_pt0[c] < mid_point);
          sB = (sub_edge.m_pt1[c] < mid_point);
          if (sA == sB)
            {
              child_boxes[sA].union_point(sub_edge.m_pt0);
              child_boxes[sA].union_point(sub_edge.m_pt1);
              child_sub_edges[sA].push_back(sub_edge);
              if (!sub_edge.m_of_closing_edge)
                {
                  ++child_num_non_closing_edges[sA];
                }
            }
          else
            {
              fastuidraw::vecN<SingleSubEdge, 2> split;

              sub_edge.split_sub_edge(c, split, mid_point);
              for(int i = 0; i < 2; ++i)
                {
                  child_boxes[i].union_point(split[i].m_pt0);
                  child_boxes[i].union_point(split[i].m_pt1);
                  child_sub_edges[i].push_back(split[i]);
                  if (!split[i].m_of_closing_edge)
                    {
                      ++child_num_non_closing_edges[i];
                    }
                }
            }
        }

      m_children[0] = FASTUIDRAWnew SubEdgeCullingHierarchy(child_boxes[0], child_sub_edges[0], child_num_non_closing_edges[0]);
      m_children[1] = FASTUIDRAWnew SubEdgeCullingHierarchy(child_boxes[1], child_sub_edges[1], child_num_non_closing_edges[1]);
    }
  else
    {
      /* steal the data */
      m_sub_edges.init(edges, num_non_closing_edges);
    }
}

SubEdgeCullingHierarchy::
~SubEdgeCullingHierarchy(void)
{
  if (m_children[0] != nullptr)
    {
      FASTUIDRAWdelete(m_children[0]);
    }

  if (m_children[1] != nullptr)
    {
      FASTUIDRAWdelete(m_children[1]);
    }
}

int
SubEdgeCullingHierarchy::
choose_splitting_coordinate(const fastuidraw::BoundingBox<float> &start_box,
                            fastuidraw::c_array<const SingleSubEdge> data,
                            float &split_value)
{
  if (data.size() < splitting_threshhold)
    {
      return -1;
    }

  fastuidraw::vecN<float, 2> split_values;
  fastuidraw::vecN<unsigned int, 2> split_counters(0, 0);
  fastuidraw::vecN<fastuidraw::uvec2, 2> child_counters(fastuidraw::uvec2(0, 0),
                                                        fastuidraw::uvec2(0, 0));
  std::vector<float> qs;
  qs.reserve(data.size());

  for(int c = 0; c < 2; ++c)
    {
      qs.clear();
      qs.push_back(start_box.min_point()[c]);
      for(const SingleSubEdge &sub_edge : data)
        {
          qs.push_back(sub_edge.m_pt0[c]);
        }
      qs.push_back(data.back().m_pt1[c]);
      qs.push_back(start_box.max_point()[c]);

      std::sort(qs.begin(), qs.end());
      split_values[c] = qs[qs.size() / 2];

      for(const SingleSubEdge &sub_edge : data)
        {
          bool sA, sB;
          sA = (sub_edge.m_pt0[c] < split_values[c]);
          sB = (sub_edge.m_pt1[c] < split_values[c]);
          if (sA != sB)
            {
              ++split_counters[c];
              ++child_counters[c][0];
              ++child_counters[c][1];
            }
          else
            {
              ++child_counters[c][sA];
            }
        }
    }

  int canidate;
  canidate = (split_counters[0] < split_counters[1]) ? 0 : 1;

  /* we require that both sides will have fewer edges
   *  than the parent size.
   */
  if (child_counters[canidate][0] < data.size() && child_counters[canidate][1] < data.size())
    {
      split_value = split_values[canidate];
      return canidate;
    }
  else
    {
      return -1;
    }
}

////////////////////////////////////////////
// StrokedPathSubset methods
StrokedPathSubset::
~StrokedPathSubset()
{
  if (m_children[0] != nullptr)
    {
      FASTUIDRAWdelete(m_children[0]);
    }
  if (m_children[1] != nullptr)
    {
      FASTUIDRAWdelete(m_children[1]);
    }
}

StrokedPathSubset*
StrokedPathSubset::
create(const SubEdgeCullingHierarchy *src,
       CreationValues &out_values)
{
  StrokedPathSubset *return_value;
  PostProcessVariables vars;

  return_value = FASTUIDRAWnew StrokedPathSubset(out_values, src);
  return_value->post_process(vars, out_values);
  return return_value;
}

void
StrokedPathSubset::
post_process(PostProcessVariables &variables, const CreationValues &constants)
{
  /* We want the depth to go in the reverse order as the
   *  draw order. The Draw order is child(0), child(1)
   *  Thus, we first handle depth child(1) and then child(0).
   */
  m_non_closing_edges.m_depth_range.m_begin = variables.m_edge_depth;
  m_closing_edges.m_depth_range.m_begin = variables.m_closing_edge_depth;

  if (have_children())
    {
      FASTUIDRAWassert(m_children[0] != nullptr);
      FASTUIDRAWassert(m_children[1] != nullptr);
      FASTUIDRAWassert(m_non_closing_edges.m_src.empty());
      FASTUIDRAWassert(m_closing_edges.m_src.empty());

      m_children[1]->post_process(variables, constants);
      m_children[0]->post_process(variables, constants);
    }
  else
    {
      FASTUIDRAWassert(m_children[0] == nullptr);
      FASTUIDRAWassert(m_children[1] == nullptr);

      variables.m_edge_depth += m_non_closing_edges.m_src.size();
      variables.m_closing_edge_depth += m_closing_edges.m_src.size();
    }
  m_non_closing_edges.m_depth_range.m_end = variables.m_edge_depth;
  m_closing_edges.m_depth_range.m_end = variables.m_closing_edge_depth;

  /* make the closing edge chunks start after the
   *  non-closing edge chunks.
   */
  m_closing_edges.m_chunk += constants.m_non_closing_edge_chunk_cnt;

  /* make vertices and indices of closing edges appear
   *  after those of non-closing edges
   */
  m_closing_edges.m_vertex_data_range += constants.m_non_closing_edge_vertex_cnt;
  m_closing_edges.m_index_data_range += constants.m_non_closing_edge_index_cnt;

  m_empty_subset = !m_non_closing_edges.non_empty()
    && !m_closing_edges.non_empty();
}

StrokedPathSubset::
StrokedPathSubset(CreationValues &out_values, const SubEdgeCullingHierarchy *src):
  m_children(nullptr, nullptr),
  m_bb(src->bounding_box())
{
  /* Draw order is:
   *    child(0)
   *    child(1)
   */
  m_non_closing_edges.m_vertex_data_range.m_begin = out_values.m_non_closing_edge_vertex_cnt;
  m_non_closing_edges.m_index_data_range.m_begin = out_values.m_non_closing_edge_index_cnt;

  m_closing_edges.m_vertex_data_range.m_begin = out_values.m_closing_edge_vertex_cnt;
  m_closing_edges.m_index_data_range.m_begin = out_values.m_closing_edge_index_cnt;

  if (src->has_children())
    {
      for(unsigned int i = 0; i < 2; ++i)
        {
          FASTUIDRAWassert(src->child(i) != nullptr);
          m_children[i] = FASTUIDRAWnew StrokedPathSubset(out_values, src->child(i));
        }
    }
  else
    {
      m_non_closing_edges.m_src = src->non_closing_edges();
      m_closing_edges.m_src = src->closing_edges();

      increment_vertices_indices(m_non_closing_edges.m_src,
                                 out_values.m_non_closing_edge_vertex_cnt,
                                 out_values.m_non_closing_edge_index_cnt);
      increment_vertices_indices(m_closing_edges.m_src,
                                 out_values.m_closing_edge_vertex_cnt,
                                 out_values.m_closing_edge_index_cnt);
    }

  m_non_closing_edges.m_vertex_data_range.m_end = out_values.m_non_closing_edge_vertex_cnt;
  m_non_closing_edges.m_index_data_range.m_end = out_values.m_non_closing_edge_index_cnt;
  m_closing_edges.m_vertex_data_range.m_end = out_values.m_closing_edge_vertex_cnt;
  m_closing_edges.m_index_data_range.m_end = out_values.m_closing_edge_index_cnt;

  m_non_closing_edges.m_chunk = out_values.m_non_closing_edge_chunk_cnt;
  m_closing_edges.m_chunk = out_values.m_closing_edge_chunk_cnt;

  ++out_values.m_non_closing_edge_chunk_cnt;
  ++out_values.m_closing_edge_chunk_cnt;
}

void
StrokedPathSubset::
increment_vertices_indices(fastuidraw::c_array<const SingleSubEdge> src,
                           unsigned int &vertex_cnt,
                           unsigned int &index_cnt)
{
  for(const SingleSubEdge &v : src)
    {
      if (v.m_has_bevel)
        {
          vertex_cnt += 3;
          index_cnt += 3;
        }

      vertex_cnt += points_per_segment;
      index_cnt += indices_per_segment_without_bevel;
    }
}

void
StrokedPathSubset::
compute_chunks(bool include_closing_edge,
               ScratchSpacePrivate &scratch,
               fastuidraw::c_array<const fastuidraw::vec3> clip_equations,
               const fastuidraw::float3x3 &clip_matrix_local,
               const fastuidraw::vec2 &recip_dimensions,
               float pixels_additional_room,
               float item_space_additional_room,
               unsigned int max_attribute_cnt,
               unsigned int max_index_cnt,
               ChunkSetPrivate &dst)
{
  scratch.m_adjusted_clip_eqs.resize(clip_equations.size());
  for(unsigned int i = 0; i < clip_equations.size(); ++i)
    {
      fastuidraw::vec3 c(clip_equations[i]);
      float f;

      /* make "w" larger by the named number of pixels.
       */
      f = fastuidraw::t_abs(c.x()) * recip_dimensions.x()
        + fastuidraw::t_abs(c.y()) * recip_dimensions.y();

      c.z() += pixels_additional_room * f;

      /* transform clip equations from clip coordinates to
       *  local coordinates.
       */
      scratch.m_adjusted_clip_eqs[i] = c * clip_matrix_local;
    }

  dst.reset();
  compute_chunks_implement(include_closing_edge,
                           scratch, item_space_additional_room,
                           max_attribute_cnt, max_index_cnt, dst);
}

void
StrokedPathSubset::
compute_chunks_take_all(bool include_closing_edge,
                        unsigned int max_attribute_cnt,
                        unsigned int max_index_cnt,
                        ChunkSetPrivate &dst)
{
  if (m_empty_subset)
    {
      return;
    }

  if (m_non_closing_edges.chunk_fits(max_attribute_cnt, max_index_cnt)
     && (!include_closing_edge || m_closing_edges.chunk_fits(max_attribute_cnt, max_index_cnt)))
    {
      dst.add_edge_chunk(m_non_closing_edges);
      if (include_closing_edge)
        {
          dst.add_edge_chunk(m_closing_edges);
        }
    }
  else if (have_children())
    {
      FASTUIDRAWassert(m_children[0] != nullptr);
      FASTUIDRAWassert(m_children[1] != nullptr);
      m_children[0]->compute_chunks_take_all(include_closing_edge, max_attribute_cnt, max_index_cnt, dst);
      m_children[1]->compute_chunks_take_all(include_closing_edge, max_attribute_cnt, max_index_cnt, dst);
    }
  else
    {
      FASTUIDRAWassert(!"Unable to fit stroked path chunk into max_attribute and max_index count limits!");
    }
}

void
StrokedPathSubset::
compute_chunks_implement(bool include_closing_edge,
                         ScratchSpacePrivate &scratch,
                         float item_space_additional_room,
                         unsigned int max_attribute_cnt,
                         unsigned int max_index_cnt,
                         ChunkSetPrivate &dst)
{
  using namespace fastuidraw;
  using namespace fastuidraw::detail;

  if (m_bb.empty() || m_empty_subset)
    {
      return;
    }

  /* clip the bounding box of this StrokedPathSubset */
  vecN<vec2, 4> bb;
  bool unclipped;

  m_bb.inflated_polygon(bb, item_space_additional_room);
  unclipped = clip_against_planes(make_c_array(scratch.m_adjusted_clip_eqs),
                                  bb, scratch.m_clipped_rect,
                                  scratch.m_clip_scratch_floats,
                                  scratch.m_clip_scratch_vec2s);
  //completely unclipped.
  if (unclipped)
    {
      compute_chunks_take_all(include_closing_edge, max_attribute_cnt, max_index_cnt, dst);
      return;
    }

  //completely clipped
  if (scratch.m_clipped_rect.empty())
    {
      return;
    }

  if (have_children())
    {
      FASTUIDRAWassert(m_children[0] != nullptr);
      FASTUIDRAWassert(m_children[1] != nullptr);
      m_children[0]->compute_chunks_implement(include_closing_edge,
                                              scratch, item_space_additional_room,
                                              max_attribute_cnt, max_index_cnt,
                                              dst);
      m_children[1]->compute_chunks_implement(include_closing_edge,
                                              scratch, item_space_additional_room,
                                              max_attribute_cnt, max_index_cnt,
                                              dst);
    }
  else
    {
      FASTUIDRAWassert(m_non_closing_edges.chunk_fits(max_attribute_cnt, max_index_cnt));
      dst.add_edge_chunk(m_non_closing_edges);

      if (include_closing_edge)
        {
          FASTUIDRAWassert(m_closing_edges.chunk_fits(max_attribute_cnt, max_index_cnt));
          dst.add_edge_chunk(m_closing_edges);
        }
    }
}

////////////////////////////////////////
// EdgeAttributeFiller methods
EdgeAttributeFiller::
EdgeAttributeFiller(const StrokedPathSubset *src,
                    const fastuidraw::TessellatedPath &P,
                    const StrokedPathSubset::CreationValues &cnts):
  m_src(src),
  m_P(P),
  m_total_vertex_cnt(cnts.m_non_closing_edge_vertex_cnt + cnts.m_closing_edge_vertex_cnt),
  m_total_index_cnt(cnts.m_non_closing_edge_index_cnt + cnts.m_closing_edge_index_cnt),
  m_total_number_chunks(cnts.m_non_closing_edge_chunk_cnt + cnts.m_closing_edge_chunk_cnt)
{
}

void
EdgeAttributeFiller::
compute_sizes(unsigned int &num_attributes,
              unsigned int &num_indices,
              unsigned int &num_attribute_chunks,
              unsigned int &num_index_chunks,
              unsigned int &number_z_ranges) const
{
  num_attribute_chunks = num_index_chunks = number_z_ranges = m_total_number_chunks;
  num_attributes = m_total_vertex_cnt;
  num_indices = m_total_index_cnt;
}

void
EdgeAttributeFiller::
fill_data(fastuidraw::c_array<fastuidraw::PainterAttribute> attribute_data,
          fastuidraw::c_array<fastuidraw::PainterIndex> index_data,
          fastuidraw::c_array<fastuidraw::c_array<const fastuidraw::PainterAttribute> > attribute_chunks,
          fastuidraw::c_array<fastuidraw::c_array<const fastuidraw::PainterIndex> > index_chunks,
          fastuidraw::c_array<fastuidraw::range_type<int> > zranges,
          fastuidraw::c_array<int> index_adjusts) const
{
  fill_data_worker(m_src, attribute_data, index_data,
                   attribute_chunks, index_chunks, zranges, index_adjusts);
}

void
EdgeAttributeFiller::
fill_data_worker(const StrokedPathSubset *e,
                 fastuidraw::c_array<fastuidraw::PainterAttribute> attribute_data,
                 fastuidraw::c_array<fastuidraw::PainterIndex> index_data,
                 fastuidraw::c_array<fastuidraw::c_array<const fastuidraw::PainterAttribute> > attribute_chunks,
                 fastuidraw::c_array<fastuidraw::c_array<const fastuidraw::PainterIndex> > index_chunks,
                 fastuidraw::c_array<fastuidraw::range_type<int> > zranges,
                 fastuidraw::c_array<int> index_adjusts) const
{
  if (e->have_children())
    {
      FASTUIDRAWassert(e->child(0) != nullptr);
      FASTUIDRAWassert(e->child(1) != nullptr);
      FASTUIDRAWassert(e->non_closing_edges().m_src.empty());
      FASTUIDRAWassert(e->closing_edges().m_src.empty());

      fill_data_worker(e->child(0), attribute_data, index_data,
                       attribute_chunks, index_chunks, zranges, index_adjusts);

      fill_data_worker(e->child(1), attribute_data, index_data,
                       attribute_chunks, index_chunks, zranges, index_adjusts);
    }

  build_chunk(e->non_closing_edges(), attribute_data, index_data,
              attribute_chunks, index_chunks, zranges, index_adjusts);

  build_chunk(e->closing_edges(), attribute_data, index_data,
              attribute_chunks, index_chunks, zranges, index_adjusts);
}

void
EdgeAttributeFiller::
build_chunk(const EdgeRanges &edge,
            fastuidraw::c_array<fastuidraw::PainterAttribute> attribute_data,
            fastuidraw::c_array<fastuidraw::PainterIndex> index_data,
            fastuidraw::c_array<fastuidraw::c_array<const fastuidraw::PainterAttribute> > attribute_chunks,
            fastuidraw::c_array<fastuidraw::c_array<const fastuidraw::PainterIndex> > index_chunks,
            fastuidraw::c_array<fastuidraw::range_type<int> > zranges,
            fastuidraw::c_array<int> index_adjusts) const
{
  fastuidraw::c_array<fastuidraw::PainterAttribute> ad;
  fastuidraw::c_array<fastuidraw::PainterIndex> id;
  unsigned int chunk;

  ad = attribute_data.sub_array(edge.m_vertex_data_range);
  id = index_data.sub_array(edge.m_index_data_range);
  chunk = edge.m_chunk;

  attribute_chunks[chunk] = ad;
  index_chunks[chunk] = id;
  index_adjusts[chunk] = -int(edge.m_vertex_data_range.m_begin);
  zranges[chunk] = fastuidraw::range_type<int>(edge.m_depth_range.m_begin,
                                               edge.m_depth_range.m_end);

  if (!edge.m_src.empty())
    {
      /* these elements are drawn AFTER the child elements,
       *  therefor they need to have a smaller depth
       */
      for(unsigned int k = 0,
            d = edge.m_depth_range.m_end - 1,
            v = edge.m_vertex_data_range.m_begin,
            i = edge.m_index_data_range.m_begin;
          k < edge.m_src.size(); ++k, --d)
        {
          process_sub_edge(edge.m_src[k], d, attribute_data, index_data, v, i);
        }
    }

  #ifdef FASTUIDRAW_DEBUG
    {
      for(unsigned int v = edge.m_vertex_data_range.m_begin;  v < edge.m_vertex_data_range.m_end; ++v)
        {
          fastuidraw::StrokedPoint P;
          fastuidraw::StrokedPoint::unpack_point(&P, attribute_data[v]);
          FASTUIDRAWassert(P.depth() >= edge.m_depth_range.m_begin);
          FASTUIDRAWassert(P.depth() < edge.m_depth_range.m_end);
        }
    }
  #endif
}

void
EdgeAttributeFiller::
process_sub_edge(const SingleSubEdge &sub_edge, unsigned int depth,
                 fastuidraw::c_array<fastuidraw::PainterAttribute> attribute_data,
                 fastuidraw::c_array<fastuidraw::PainterIndex> indices,
                 unsigned int &vert_offset, unsigned int &index_offset) const
{
  const int boundary_values[3] = { 1, 1, 0 };
  const float normal_sign[3] = { 1.0f, -1.0f, 0.0f };
  fastuidraw::vecN<fastuidraw::StrokedPoint, 6> pts;

  if (sub_edge.m_has_bevel)
    {
      indices[index_offset + 0] = vert_offset + 0;
      indices[index_offset + 1] = vert_offset + 1;
      indices[index_offset + 2] = vert_offset + 2;
      index_offset += 3;

      for(unsigned int k = 0; k < 3; ++k)
        {
          pts[k].m_position = sub_edge.m_pt0;
          pts[k].m_distance_from_edge_start = sub_edge.m_distance_from_edge_start;
          pts[k].m_distance_from_contour_start = sub_edge.m_distance_from_contour_start;
          pts[k].m_edge_length = sub_edge.m_edge_length;
          pts[k].m_open_contour_length = sub_edge.m_open_contour_length;
          pts[k].m_closed_contour_length = sub_edge.m_closed_contour_length;
          pts[k].m_auxiliary_offset = fastuidraw::vec2(0.0f, 0.0f);
        }

      pts[0].m_pre_offset = fastuidraw::vec2(0.0f, 0.0f);
      pts[0].m_packed_data = pack_data(0, fastuidraw::StrokedPoint::offset_start_sub_edge, depth)
        | fastuidraw::StrokedPoint::bevel_edge_mask;

      pts[1].m_pre_offset = sub_edge.m_bevel_lambda * sub_edge.m_bevel_normal;
      pts[1].m_packed_data = pack_data(1, fastuidraw::StrokedPoint::offset_start_sub_edge, depth)
        | fastuidraw::StrokedPoint::bevel_edge_mask;

      pts[2].m_pre_offset = sub_edge.m_bevel_lambda * sub_edge.m_normal;
      pts[2].m_packed_data = pack_data(1, fastuidraw::StrokedPoint::offset_start_sub_edge, depth)
        | fastuidraw::StrokedPoint::bevel_edge_mask;

      for(unsigned int i = 0; i < 3; ++i)
        {
          pts[i].fastuidraw::StrokedPoint::pack_point(&attribute_data[vert_offset + i]);
        }

      vert_offset += 3;
    }

  /* The quad is:
   *  (p, n, delta,  1),
   *  (p,-n, delta,  1),
   *  (p, 0,     0,  0),
   *  (p_next,  n, -delta, 1),
   *  (p_next, -n, -delta, 1),
   *  (p_next,  0, 0)
   *
   *  Notice that we are encoding if it is
   *  start or end of edge from the sign of
   *  m_on_boundary.
   */
  for(unsigned int k = 0; k < 3; ++k)
    {
      pts[k].m_position = sub_edge.m_pt0;
      pts[k].m_distance_from_edge_start = sub_edge.m_distance_from_edge_start;
      pts[k].m_distance_from_contour_start = sub_edge.m_distance_from_contour_start;
      pts[k].m_edge_length = sub_edge.m_edge_length;
      pts[k].m_open_contour_length = sub_edge.m_open_contour_length;
      pts[k].m_closed_contour_length = sub_edge.m_closed_contour_length;
      pts[k].m_pre_offset = normal_sign[k] * sub_edge.m_normal;
      pts[k].m_auxiliary_offset = sub_edge.m_delta;
      pts[k].m_packed_data = pack_data(boundary_values[k],
                                       fastuidraw::StrokedPoint::offset_start_sub_edge,
                                       depth);

      pts[k + 3].m_position = sub_edge.m_pt1;
      pts[k + 3].m_distance_from_edge_start = sub_edge.m_distance_from_edge_start + sub_edge.m_sub_edge_length;
      pts[k + 3].m_distance_from_contour_start = sub_edge.m_distance_from_contour_start + sub_edge.m_sub_edge_length;
      pts[k + 3].m_edge_length = sub_edge.m_edge_length;
      pts[k + 3].m_open_contour_length = sub_edge.m_open_contour_length;
      pts[k + 3].m_closed_contour_length = sub_edge.m_closed_contour_length;
      pts[k + 3].m_pre_offset = normal_sign[k] * sub_edge.m_normal;
      pts[k + 3].m_auxiliary_offset = -sub_edge.m_delta;
      pts[k + 3].m_packed_data = pack_data(boundary_values[k],
                                           fastuidraw::StrokedPoint::offset_end_sub_edge,
                                           depth);
    }

  for(unsigned int i = 0; i < 6; ++i)
    {
      pts[i].fastuidraw::StrokedPoint::pack_point(&attribute_data[vert_offset + i]);
    }

  indices[index_offset + 0] = vert_offset + 0;
  indices[index_offset + 1] = vert_offset + 2;
  indices[index_offset + 2] = vert_offset + 5;
  indices[index_offset + 3] = vert_offset + 0;
  indices[index_offset + 4] = vert_offset + 5;
  indices[index_offset + 5] = vert_offset + 3;

  indices[index_offset + 6] = vert_offset + 2;
  indices[index_offset + 7] = vert_offset + 1;
  indices[index_offset + 8] = vert_offset + 4;
  indices[index_offset + 9] = vert_offset + 2;
  indices[index_offset + 10] = vert_offset + 4;
  indices[index_offset + 11] = vert_offset + 5;

  index_offset += StrokedPathSubset::indices_per_segment_without_bevel;
  vert_offset += StrokedPathSubset::points_per_segment;
}

/////////////////////////////////////////////
// StrokedPathPrivate methods
StrokedPathPrivate::
StrokedPathPrivate(const fastuidraw::TessellatedPath &P,
                   const fastuidraw::StrokedCapsJoins::Builder &b):
  m_caps_joins(b),
  m_subset(nullptr)
{
  if (!P.segment_data().empty())
    {
      m_empty_path = false;
      create_edges(P);
    }
  else
    {
      m_empty_path = true;
      std::fill(m_chunk_of_edges.begin(), m_chunk_of_edges.end(), 0);
    }
}

StrokedPathPrivate::
~StrokedPathPrivate()
{
  if (!m_empty_path)
    {
      FASTUIDRAWdelete(m_subset);
    }
}

void
StrokedPathPrivate::
ready_builder(const fastuidraw::TessellatedPath *tess,
              fastuidraw::StrokedCapsJoins::Builder &b)
{
  for(unsigned int c = 0; c < tess->number_contours(); ++c)
    {
      ready_builder_contour(tess, c, b);
    }
}

void
StrokedPathPrivate::
ready_builder_contour(const fastuidraw::TessellatedPath *tess,
                      unsigned int c,
                      fastuidraw::StrokedCapsJoins::Builder &b)
{
  fastuidraw::c_array<const fastuidraw::TessellatedPath::segment> last_segs;
  fastuidraw::vec2 delta, last_direction;
  float delta_mag;
  const float tol(0.000001f);

  last_segs = tess->edge_segment_data(c, 0);

  delta = last_segs.front().m_data - last_segs.front().m_p;
  delta_mag = delta.magnitude();
  if (delta_mag < tol)
    {
      delta = fastuidraw::vec2(1.0f, 0.0f);
    }
  else
    {
      delta /= delta_mag;
    }
  last_direction = delta;
  b.begin_contour(last_segs.front().m_p, delta);

  for(unsigned int e = 1, ende = tess->number_edges(c); e < ende; ++e)
    {
      fastuidraw::c_array<const fastuidraw::TessellatedPath::segment> segs;
      fastuidraw::vec2 delta_into, delta_leaving;
      float mag;

      segs = tess->edge_segment_data(c, e);
      delta_into = last_segs.back().m_data - last_segs.back().m_p;
      mag = delta_into.magnitude();
      if (mag < tol)
        {
          delta_into = last_direction;
        }
      else
        {
          delta_into /= mag;
          last_direction = delta_into;
        }

      delta_leaving = segs.front().m_data - segs.front().m_p;
      mag = delta_leaving.magnitude();
      if (mag < tol)
        {
          delta_leaving = last_direction;
        }
      else
        {
          delta_leaving /= mag;
          last_direction = delta_leaving;
        }

      b.add_join(segs.front().m_p,
                 last_segs.back().m_edge_length,
                 delta_into, delta_leaving);
      last_segs = segs;
    }

  delta = last_segs.back().m_data - last_segs.back().m_p;
  delta_mag = delta.magnitude();
  if (delta_mag < tol)
    {
      delta = last_direction;
    }
  else
    {
      delta /= delta_mag;
    }
  b.end_contour(last_segs.back().m_edge_length, delta);
}

void
StrokedPathPrivate::
create_edges(const fastuidraw::TessellatedPath &P)
{
  SubEdgeCullingHierarchy *s;
  StrokedPathSubset::CreationValues cnts;

  FASTUIDRAWassert(!m_empty_path);
  s = SubEdgeCullingHierarchy::create(P);
  m_subset = StrokedPathSubset::create(s, cnts);
  m_edges.set_data(EdgeAttributeFiller(m_subset, P, cnts));

  /* the chunks of the root element have the data for everything.
   */
  m_chunk_of_edges[fastuidraw::StrokedPath::all_non_closing] = m_subset->non_closing_edges().m_chunk;
  m_chunk_of_edges[fastuidraw::StrokedPath::all_closing] = m_subset->closing_edges().m_chunk;

  FASTUIDRAWdelete(s);
}

//////////////////////////////////////////////
// fastuidraw::StrokedPath::ScratchSpace methods
fastuidraw::StrokedPath::ScratchSpace::
ScratchSpace(void)
{
  m_d = FASTUIDRAWnew ScratchSpacePrivate();
}

fastuidraw::StrokedPath::ScratchSpace::
~ScratchSpace(void)
{
  ScratchSpacePrivate *d;
  d = static_cast<ScratchSpacePrivate*>(m_d);
  FASTUIDRAWdelete(d);
  m_d = nullptr;
}

///////////////////////////////////////
// ChunkSetPrivate methods
void
ChunkSetPrivate::
add_edge_chunk(const EdgeRanges &ed)
{
  if (ed.non_empty())
    {
      m_edge_chunks.push_back(ed.m_chunk);
    }
}

//////////////////////////////////////////
// fastuidraw::StrokedPath::ChunkSet methods
fastuidraw::StrokedPath::ChunkSet::
ChunkSet(void)
{
  m_d = FASTUIDRAWnew ChunkSetPrivate();
}

fastuidraw::StrokedPath::ChunkSet::
~ChunkSet(void)
{
  ChunkSetPrivate *d;
  d = static_cast<ChunkSetPrivate*>(m_d);
  FASTUIDRAWdelete(d);
}

fastuidraw::c_array<const unsigned int>
fastuidraw::StrokedPath::ChunkSet::
edge_chunks(void) const
{
  ChunkSetPrivate *d;
  d = static_cast<ChunkSetPrivate*>(m_d);
  return d->edge_chunks();
}

//////////////////////////////////////////////////////////////
// fastuidraw::StrokedPath methods
fastuidraw::StrokedPath::
StrokedPath(const fastuidraw::TessellatedPath &P)
{
  StrokedCapsJoins::Builder b;
  StrokedPathPrivate::ready_builder(&P, b);
  m_d = FASTUIDRAWnew StrokedPathPrivate(P, b);
}

fastuidraw::StrokedPath::
~StrokedPath()
{
  StrokedPathPrivate *d;
  d = static_cast<StrokedPathPrivate*>(m_d);
  FASTUIDRAWdelete(d);
  m_d = nullptr;
}

void
fastuidraw::StrokedPath::
compute_chunks(ScratchSpace &scratch_space,
               c_array<const vec3> clip_equations,
               const float3x3 &clip_matrix_local,
               const vec2 &recip_dimensions,
               float pixels_additional_room,
               float item_space_additional_room,
               bool include_closing_edges,
               unsigned int max_attribute_cnt,
               unsigned int max_index_cnt,
               ChunkSet &dst) const
{
  StrokedPathPrivate *d;
  ScratchSpacePrivate *scratch_space_ptr;
  ChunkSetPrivate *chunk_set_ptr;

  d = static_cast<StrokedPathPrivate*>(m_d);
  scratch_space_ptr = static_cast<ScratchSpacePrivate*>(scratch_space.m_d);
  chunk_set_ptr = static_cast<ChunkSetPrivate*>(dst.m_d);

  if (d->m_empty_path)
    {
      chunk_set_ptr->reset();
      return;
    }

  d->m_subset->compute_chunks(include_closing_edges,
                              *scratch_space_ptr,
                              clip_equations,
                              clip_matrix_local,
                              recip_dimensions,
                              pixels_additional_room,
                              item_space_additional_room,
                              max_attribute_cnt,
                              max_index_cnt,
                              *chunk_set_ptr);
}

const fastuidraw::PainterAttributeData&
fastuidraw::StrokedPath::
edges(void) const
{
  StrokedPathPrivate *d;
  d = static_cast<StrokedPathPrivate*>(m_d);
  return d->m_edges;
}

unsigned int
fastuidraw::StrokedPath::
chunk_of_edges(enum chunk_selection c) const
{
  StrokedPathPrivate *d;
  d = static_cast<StrokedPathPrivate*>(m_d);
  return d->m_chunk_of_edges[c];
}

const fastuidraw::StrokedCapsJoins&
fastuidraw::StrokedPath::
caps_joins(void) const
{
  StrokedPathPrivate *d;
  d = static_cast<StrokedPathPrivate*>(m_d);
  return d->m_caps_joins;
}
