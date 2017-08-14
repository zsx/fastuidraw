/*!
 * \file font_freetype.cpp
 * \brief file font_freetype.cpp
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

#include <sstream>
#include <fastuidraw/text/font_freetype.hpp>
#include <fastuidraw/text/glyph_layout_data.hpp>
#include <fastuidraw/text/glyph_render_data.hpp>
#include <fastuidraw/text/glyph_render_data_curve_pair.hpp>
#include <fastuidraw/text/glyph_render_data_distance_field.hpp>
#include <fastuidraw/text/glyph_render_data_coverage.hpp>

#include "../private/array2d.hpp"
#include "../private/util_private.hpp"
#include "../private/int_path.hpp"

#include <ft2build.h>
#include FT_OUTLINE_H

namespace
{
  /* covert TO FontCoordinates from Freetype values that
     are stored in scaled pixel size values.
   */
  class font_coordinate_converter
  {
  public:
    font_coordinate_converter(FT_Face face, unsigned int pixel_size)
    {
      m_factor = static_cast<float>(face->units_per_EM);
      m_factor /= 64.0f * static_cast<float>(pixel_size);
    }

    font_coordinate_converter(void):
      m_factor(1.0f)
    {}

    float
    factor(void) const
    {
      return m_factor;
    }

    float
    operator()(FT_Pos p) const
    {
      return static_cast<float>(p) * m_factor;
    }

  private:
    float m_factor;
  };

  class RenderParamsPrivate
  {
  public:
    RenderParamsPrivate(void):
      m_distance_field_pixel_size(48),
      m_distance_field_max_distance(96.0f),
      m_curve_pair_pixel_size(32)
    {}

    unsigned int m_distance_field_pixel_size;
    float m_distance_field_max_distance;
    unsigned int m_curve_pair_pixel_size;
  };

  class IntPathCreator
  {
  public:
    static
    void
    decompose_to_path(FT_Outline *outline, fastuidraw::detail::IntPath &p)
    {
      IntPathCreator datum(p);
      FT_Outline_Funcs funcs;

      funcs.move_to = &ft_outline_move_to;
      funcs.line_to = &ft_outline_line_to;
      funcs.conic_to = &ft_outline_conic_to;
      funcs.cubic_to = &ft_outline_cubic_to;
      funcs.shift = 0;
      funcs.delta = 0;
      FT_Outline_Decompose(outline, &funcs, &datum);
    }

    static
    void
    decompose_to_path(FT_Outline *outline, fastuidraw::Path &p,
                      font_coordinate_converter C)
    {
      fastuidraw::detail::IntPath ip;
      fastuidraw::detail::IntBezierCurve::transformation<float> tr(C.factor());

      decompose_to_path(outline, ip);
      ip.add_to_path(tr, &p);
    }

  private:
    explicit
    IntPathCreator(fastuidraw::detail::IntPath &P):
      m_path(P)
    {}

    static
    int
    ft_outline_move_to(const FT_Vector *pt, void *user)
    {
      IntPathCreator *p;
      p = static_cast<IntPathCreator*>(user);
      p->m_path.move_to(fastuidraw::ivec2(pt->x, pt->y));
      return 0;
    }

    static
    int
    ft_outline_line_to(const FT_Vector *pt, void *user)
    {
      IntPathCreator *p;
      p = static_cast<IntPathCreator*>(user);
      p->m_path.line_to(fastuidraw::ivec2(pt->x, pt->y));
      return 0;
    }

    static
    int
    ft_outline_conic_to(const FT_Vector *control_pt,
                        const FT_Vector *pt, void *user)
    {
      IntPathCreator *p;
      p = static_cast<IntPathCreator*>(user);
      p->m_path.conic_to(fastuidraw::ivec2(control_pt->x, control_pt->y),
                         fastuidraw::ivec2(pt->x, pt->y));
      return 0;
    }

    static
    int
    ft_outline_cubic_to(const FT_Vector *control_pt0,
                        const FT_Vector *control_pt1,
                        const FT_Vector *pt, void *user)
    {
      IntPathCreator *p;
      p = static_cast<IntPathCreator*>(user);
      p->m_path.cubic_to(fastuidraw::ivec2(control_pt0->x, control_pt0->y),
                         fastuidraw::ivec2(control_pt1->x, control_pt1->y),
                         fastuidraw::ivec2(pt->x, pt->y));
      return 0;
    }

    fastuidraw::detail::IntPath &m_path;
  };

  class FontFreeTypePrivate
  {
  public:

    class FaceGrabber
    {
    public:
      FaceGrabber(FontFreeTypePrivate *q);
      ~FaceGrabber();

      fastuidraw::FreetypeFace *m_p;
    };

    FontFreeTypePrivate(fastuidraw::FontFreeType *p,
                        const fastuidraw::reference_counted_ptr<fastuidraw::FreetypeFace::GeneratorBase> &pface_generator,
                        fastuidraw::reference_counted_ptr<fastuidraw::FreetypeLib> lib,
                        const fastuidraw::FontFreeType::RenderParams &render_params);

    ~FontFreeTypePrivate();

    static
    void
    common_compute_rendering_data(FT_Face face, fastuidraw::FontFreeType *p,
                                  font_coordinate_converter C, FT_Int32 load_flags,
                                  fastuidraw::GlyphLayoutData &layout,
                                  uint32_t glyph_code);

    void
    compute_rendering_data(int pixel_size, uint32_t glyph_code,
                           fastuidraw::GlyphLayoutData &layout,
                           fastuidraw::GlyphRenderDataCoverage &output,
                           fastuidraw::Path &path);

    void
    compute_rendering_data(uint32_t glyph_code,
                           fastuidraw::GlyphLayoutData &layout,
                           fastuidraw::GlyphRenderDataDistanceField &output,
                           fastuidraw::Path &path);

    void
    compute_rendering_data(uint32_t glyph_code,
                           fastuidraw::GlyphLayoutData &layout,
                           fastuidraw::GlyphRenderDataCurvePair &output,
                           fastuidraw::Path &path);

    fastuidraw::reference_counted_ptr<fastuidraw::FreetypeFace::GeneratorBase> m_generator;
    fastuidraw::FontFreeType::RenderParams m_render_params;
    fastuidraw::reference_counted_ptr<fastuidraw::FreetypeLib> m_lib;
    fastuidraw::FontFreeType *m_p;

    /* for now we have a static number of m_faces we use for parallel
       glyph generation
     */
    fastuidraw::vecN<fastuidraw::reference_counted_ptr<fastuidraw::FreetypeFace>, 8> m_faces;
  };
}

///////////////////////////////////////////
// FontFreeTypePrivate::FaceGrabber methods
FontFreeTypePrivate::FaceGrabber::
FaceGrabber(FontFreeTypePrivate *q):
  m_p(nullptr)
{
  while(!m_p)
    {
      for(unsigned int i = 0; i < q->m_faces.size() && !m_p; ++i)
        {
          const fastuidraw::reference_counted_ptr<fastuidraw::FreetypeFace> &face(q->m_faces[i]);
          if(face->try_lock())
            {
              m_p = face.get();
            }
        }
    }
}

FontFreeTypePrivate::FaceGrabber::
~FaceGrabber()
{
  m_p->unlock();
}

//////////////////////////////////////////////////
// FontFreeTypePrivate methods
FontFreeTypePrivate::
FontFreeTypePrivate(fastuidraw::FontFreeType *p,
                    const fastuidraw::reference_counted_ptr<fastuidraw::FreetypeFace::GeneratorBase> &generator,
                    fastuidraw::reference_counted_ptr<fastuidraw::FreetypeLib> lib,
                    const fastuidraw::FontFreeType::RenderParams &render_params):
  m_generator(generator),
  m_render_params(render_params),
  m_lib(lib),
  m_p(p)
{
  if(!m_lib)
    {
      m_lib = FASTUIDRAWnew fastuidraw::FreetypeLib();
    }
  for(unsigned int i = 0; i < m_faces.size(); ++i)
    {
      m_faces[i] = m_generator->create_face(m_lib);
      FT_Set_Transform(m_faces[i]->face(), nullptr, nullptr);
    }
}

FontFreeTypePrivate::
~FontFreeTypePrivate()
{
}

void
FontFreeTypePrivate::
common_compute_rendering_data(FT_Face face, fastuidraw::FontFreeType *p,
                              font_coordinate_converter C, FT_Int32 load_flags,
                              fastuidraw::GlyphLayoutData &output,
                              uint32_t glyph_code)
{
  FT_Load_Glyph(face, glyph_code, load_flags);
  output.m_size.x() = C(face->glyph->metrics.width);
  output.m_size.y() = C(face->glyph->metrics.height);
  output.m_horizontal_layout_offset.x() = C(face->glyph->metrics.horiBearingX);
  output.m_horizontal_layout_offset.y() = C(face->glyph->metrics.horiBearingY) - output.m_size.y();
  output.m_vertical_layout_offset.x() = C(face->glyph->metrics.vertBearingX);
  output.m_vertical_layout_offset.y() = C(face->glyph->metrics.vertBearingY) - output.m_size.y();
  output.m_advance.x() = C(face->glyph->metrics.horiAdvance);
  output.m_advance.y() = C(face->glyph->metrics.vertAdvance);
  output.m_glyph_code = glyph_code;
  output.m_units_per_EM = face->units_per_EM;
  output.m_font = p;
}

void
FontFreeTypePrivate::
compute_rendering_data(int pixel_size, uint32_t glyph_code,
                       fastuidraw::GlyphLayoutData &layout,
                       fastuidraw::GlyphRenderDataCoverage &output,
                       fastuidraw::Path &path)
{
  FaceGrabber p(this);

  FT_Face face(p.m_p->face());
  fastuidraw::ivec2 bitmap_sz;
  font_coordinate_converter C(face, pixel_size);

  FT_Set_Pixel_Sizes(face, pixel_size, pixel_size);
  common_compute_rendering_data(face, m_p, C, FT_LOAD_DEFAULT, layout, glyph_code);
  IntPathCreator::decompose_to_path(&face->glyph->outline, path, C);
  FT_Render_Glyph(face->glyph, FT_RENDER_MODE_NORMAL);

  bitmap_sz.x() = face->glyph->bitmap.width;
  bitmap_sz.y() = face->glyph->bitmap.rows;

  /* add one pixel slack on glyph
   */
  if(bitmap_sz.x() != 0 && bitmap_sz.y() != 0)
    {
      int pitch;

      pitch = face->glyph->bitmap.pitch;
      output.resize(bitmap_sz + fastuidraw::ivec2(1, 1));
      std::fill(output.coverage_values().begin(), output.coverage_values().end(), 0);
      for(int y = 0; y < bitmap_sz.y(); ++y)
        {
          for(int x = 0; x < bitmap_sz.x(); ++x)
            {
              int write_location, read_location;

              write_location = x + y * output.resolution().x();
              read_location = x + (bitmap_sz.y() - 1 - y) * pitch;
              output.coverage_values()[write_location] = face->glyph->bitmap.buffer[read_location];
            }
        }
    }
  else
    {
      output.resize(fastuidraw::ivec2(0, 0));
    }
}

void
FontFreeTypePrivate::
compute_rendering_data(uint32_t glyph_code,
                       fastuidraw::GlyphLayoutData &layout,
                       fastuidraw::GlyphRenderDataDistanceField &output,
                       fastuidraw::Path &path)
{
  fastuidraw::detail::IntPath int_path_ecm;
  int units_per_EM;
  fastuidraw::ivec2 layout_offset;
  int outline_flags;


  {
    FaceGrabber p(this);
    FT_Face face(p.m_p->face());

    common_compute_rendering_data(face, m_p, font_coordinate_converter(),
                                  FT_LOAD_NO_SCALE | FT_LOAD_NO_HINTING
                                  | FT_LOAD_NO_BITMAP | FT_LOAD_IGNORE_TRANSFORM
                                  | FT_LOAD_LINEAR_DESIGN,
                                  layout, glyph_code);
    units_per_EM = face->units_per_EM;
    outline_flags = face->glyph->outline.flags;
    layout_offset = fastuidraw::ivec2(face->glyph->metrics.horiBearingX,
                                      face->glyph->metrics.horiBearingY);
    layout_offset.y() -= face->glyph->metrics.height;
    IntPathCreator::decompose_to_path(&face->glyph->outline, int_path_ecm);
  }

  if(int_path_ecm.empty())
    {
      return;
    }

  fastuidraw::detail::IntBezierCurve::transformation<float> identity_tr;
  int_path_ecm.add_to_path(identity_tr, &path);

  /* choose the correct fill rule as according to outline_flags */
  enum fastuidraw::PainterEnums::fill_rule_t fill_rule;
  fill_rule = (outline_flags & FT_OUTLINE_EVEN_ODD_FILL) ?
    fastuidraw::PainterEnums::odd_even_fill_rule:
    fastuidraw::PainterEnums::nonzero_fill_rule;

  /* compute the step value needed to create the distance field value*/
  int pixel_size(m_render_params.distance_field_pixel_size());
  float scale_factor(static_cast<float>(pixel_size) / static_cast<float>(units_per_EM));

  /* compute how many pixels we need to store the glyph. */
  fastuidraw::vec2 image_sz_f(layout.m_size * scale_factor);
  fastuidraw::ivec2 image_sz(ceilf(image_sz_f.x()), ceilf(image_sz_f.y()));

  if(image_sz.x() == 0 || image_sz.y() == 0)
    {
      output.resize(fastuidraw::ivec2(0, 0));
      return;
    }

  /* we translate by -layout_offset to make the points of
     the IntPath match correctly with that of the texel
     data (this also gaurantees that the box of the glyph
     is (0, 0)). In addition, we scale by 2 * pixel_size;
     By doing this, the distance between texel-centers
     is then just 2 * units_per_EM.
  */
  int tr_scale(2 * pixel_size);
  fastuidraw::ivec2 tr_translate(-2 * pixel_size * layout_offset);
  fastuidraw::detail::IntBezierCurve::transformation<int> tr(tr_scale, tr_translate);
  fastuidraw::ivec2 texel_distance(2 * units_per_EM);
  float max_distance = (m_render_params.distance_field_max_distance() / 64.0f)
    * static_cast<float>(2 * units_per_EM);

  int_path_ecm.extract_render_data(texel_distance, image_sz, max_distance, tr,
                                   fastuidraw::CustomFillRuleFunction(fill_rule),
                                   &output);
}

void
FontFreeTypePrivate::
compute_rendering_data(uint32_t glyph_code,
                       fastuidraw::GlyphLayoutData &layout,
                       fastuidraw::GlyphRenderDataCurvePair &output,
                       fastuidraw::Path &path)
{
  fastuidraw::detail::IntPath int_path_ecm;
  int units_per_EM;
  fastuidraw::ivec2 layout_offset;
  int outline_flags;

  {
    FaceGrabber p(this);
    FT_Face face(p.m_p->face());
    common_compute_rendering_data(face, m_p,
                                  font_coordinate_converter(),
                                  FT_LOAD_NO_SCALE | FT_LOAD_NO_HINTING
                                  | FT_LOAD_NO_BITMAP | FT_LOAD_IGNORE_TRANSFORM
                                  | FT_LOAD_LINEAR_DESIGN,
                                  layout, glyph_code);
    units_per_EM = face->units_per_EM;
    outline_flags = face->glyph->outline.flags;
    layout_offset = fastuidraw::ivec2(face->glyph->metrics.horiBearingX,
                                      face->glyph->metrics.horiBearingY);
    layout_offset.y() -= face->glyph->metrics.height;
    IntPathCreator::decompose_to_path(&face->glyph->outline, int_path_ecm);
  }

  /* choose the correct fill rule as according to outline_flags */
  enum fastuidraw::PainterEnums::fill_rule_t fill_rule;
  fill_rule = (outline_flags & FT_OUTLINE_EVEN_ODD_FILL) ?
    fastuidraw::PainterEnums::odd_even_fill_rule:
    fastuidraw::PainterEnums::nonzero_fill_rule;

  int pixel_size(m_render_params.curve_pair_pixel_size());
  float scale_factor(static_cast<float>(pixel_size) / static_cast<float>(units_per_EM));

  /* compute how many pixels we need to store the glyph. */
  fastuidraw::vec2 image_sz_f(layout.m_size * scale_factor);
  fastuidraw::ivec2 image_sz(ceilf(image_sz_f.x()), ceilf(image_sz_f.y()));

  /* Use the same transformation as the DistanceField case */
  int tr_scale(2 * pixel_size);
  fastuidraw::ivec2 tr_translate(-2 * pixel_size * layout_offset);
  fastuidraw::detail::IntBezierCurve::transformation<int> tr(tr_scale, tr_translate);
  fastuidraw::ivec2 texel_distance(2 * units_per_EM);
  float curvature_collapse(0.05f);

  int_path_ecm.filter(curvature_collapse, tr, texel_distance);
  if(int_path_ecm.empty() || image_sz.x() == 0 || image_sz.y() == 0)
    {
      output.resize_active_curve_pair(fastuidraw::ivec2(0, 0));
      return;
    }

  /* extract to a Path */
  fastuidraw::detail::IntBezierCurve::transformation<float> identity_tr;
  int_path_ecm.add_to_path(identity_tr, &path);

  /* extract render data*/
  int_path_ecm.extract_render_data(texel_distance, image_sz, tr,
                                   fastuidraw::CustomFillRuleFunction(fill_rule),
                                    &output);
}

/////////////////////////////////////////////
// fastuidraw::FontFreeType::RenderParams methods
fastuidraw::FontFreeType::RenderParams::
RenderParams(void)
{
  m_d = FASTUIDRAWnew RenderParamsPrivate();
}

fastuidraw::FontFreeType::RenderParams::
RenderParams(const RenderParams &obj)
{
  RenderParamsPrivate *obj_d;
  obj_d = static_cast<RenderParamsPrivate*>(obj.m_d);
  m_d = FASTUIDRAWnew RenderParamsPrivate(*obj_d);
}

fastuidraw::FontFreeType::RenderParams::
~RenderParams(void)
{
  RenderParamsPrivate *d;
  d = static_cast<RenderParamsPrivate*>(m_d);
  FASTUIDRAWdelete(d);
  m_d = nullptr;
}

void
fastuidraw::FontFreeType::RenderParams::
swap(RenderParams &obj)
{
  std::swap(obj.m_d, m_d);
}

fastuidraw::FontFreeType::RenderParams&
fastuidraw::FontFreeType::RenderParams::
operator=(const RenderParams &rhs)
{
  if(&rhs != this)
    {
      RenderParams v(rhs);
      swap(v);
    }
  return *this;
}

fastuidraw::FontFreeType::RenderParams&
fastuidraw::FontFreeType::RenderParams::
distance_field_pixel_size(unsigned int v)
{
  RenderParamsPrivate *d;
  d = static_cast<RenderParamsPrivate*>(m_d);
  d->m_distance_field_pixel_size = v;
  return *this;
}

unsigned int
fastuidraw::FontFreeType::RenderParams::
distance_field_pixel_size(void) const
{
  RenderParamsPrivate *d;
  d = static_cast<RenderParamsPrivate*>(m_d);
  return d->m_distance_field_pixel_size;
}

fastuidraw::FontFreeType::RenderParams&
fastuidraw::FontFreeType::RenderParams::
distance_field_max_distance(float v)
{
  RenderParamsPrivate *d;
  d = static_cast<RenderParamsPrivate*>(m_d);
  d->m_distance_field_max_distance = v;
  return *this;
}

float
fastuidraw::FontFreeType::RenderParams::
distance_field_max_distance(void) const
{
  RenderParamsPrivate *d;
  d = static_cast<RenderParamsPrivate*>(m_d);
  return d->m_distance_field_max_distance;
}

fastuidraw::FontFreeType::RenderParams&
fastuidraw::FontFreeType::RenderParams::
curve_pair_pixel_size(unsigned int v)
{
  RenderParamsPrivate *d;
  d = static_cast<RenderParamsPrivate*>(m_d);
  d->m_curve_pair_pixel_size = v;
  return *this;
}

unsigned int
fastuidraw::FontFreeType::RenderParams::
curve_pair_pixel_size(void) const
{
  RenderParamsPrivate *d;
  d = static_cast<RenderParamsPrivate*>(m_d);
  return d->m_curve_pair_pixel_size;
}

///////////////////////////////////////////////////
// fastuidraw::FontFreeType methods
fastuidraw::FontFreeType::
FontFreeType(const reference_counted_ptr<FreetypeFace::GeneratorBase> &pface_generator,
             const FontProperties &props, const RenderParams &render_params,
             const reference_counted_ptr<FreetypeLib> &lib):
  FontBase(props)
{
  m_d = FASTUIDRAWnew FontFreeTypePrivate(this, pface_generator, lib, render_params);
}

fastuidraw::FontFreeType::
FontFreeType(const reference_counted_ptr<FreetypeFace::GeneratorBase> &pface_generator,
             const RenderParams &render_params,
             const reference_counted_ptr<FreetypeLib> &lib):
  FontBase(compute_font_properties_from_face(pface_generator->create_face(lib)->face()))
{
  m_d = FASTUIDRAWnew FontFreeTypePrivate(this, pface_generator, lib, render_params);
}

fastuidraw::FontFreeType::
~FontFreeType()
{
  FontFreeTypePrivate *d;
  d = static_cast<FontFreeTypePrivate*>(m_d);
  FASTUIDRAWdelete(d);
  m_d = nullptr;
}

uint32_t
fastuidraw::FontFreeType::
glyph_code(uint32_t pcharacter_code) const
{
  FontFreeTypePrivate *d;
  d = static_cast<FontFreeTypePrivate*>(m_d);

  FT_UInt glyphcode;
  FontFreeTypePrivate::FaceGrabber p(d);
  glyphcode = FT_Get_Char_Index(p.m_p->face(), FT_ULong(pcharacter_code));

  return glyphcode;
}

bool
fastuidraw::FontFreeType::
can_create_rendering_data(enum glyph_type tp) const
{
  return tp == coverage_glyph
    || tp == distance_field_glyph
    || tp == curve_pair_glyph;
}

fastuidraw::GlyphRenderData*
fastuidraw::FontFreeType::
compute_rendering_data(GlyphRender render, uint32_t glyph_code,
                       GlyphLayoutData &layout, Path &path) const
{
  FontFreeTypePrivate *d;
  d = static_cast<FontFreeTypePrivate*>(m_d);

  switch(render.m_type)
    {
    case coverage_glyph:
      {
        GlyphRenderDataCoverage *data;
        data = FASTUIDRAWnew GlyphRenderDataCoverage();
        d->compute_rendering_data(render.m_pixel_size, glyph_code, layout, *data, path);
        return data;
      }
      break;

    case distance_field_glyph:
      {
        GlyphRenderDataDistanceField *data;
        data = FASTUIDRAWnew GlyphRenderDataDistanceField();
        d->compute_rendering_data(glyph_code, layout, *data, path);
        return data;
      }
      break;

    case curve_pair_glyph:
      {
        GlyphRenderDataCurvePair *data;
        data = FASTUIDRAWnew GlyphRenderDataCurvePair();
        d->compute_rendering_data(glyph_code, layout, *data, path);
        return data;
      }
      break;

    default:
      FASTUIDRAWassert(!"Invalid glyph type");
      return nullptr;
    }
}

const fastuidraw::FontFreeType::RenderParams&
fastuidraw::FontFreeType::
render_params(void) const
{
  FontFreeTypePrivate *d;
  d = static_cast<FontFreeTypePrivate*>(m_d);
  return d->m_render_params;
}

const fastuidraw::reference_counted_ptr<fastuidraw::FreetypeFace::GeneratorBase>&
fastuidraw::FontFreeType::
face_generator(void) const
{
  FontFreeTypePrivate *d;
  d = static_cast<FontFreeTypePrivate*>(m_d);
  return d->m_generator;
}

const fastuidraw::reference_counted_ptr<fastuidraw::FreetypeLib>&
fastuidraw::FontFreeType::
lib(void) const
{
  FontFreeTypePrivate *d;
  d = static_cast<FontFreeTypePrivate*>(m_d);
  return d->m_lib;
}

int
fastuidraw::FontFreeType::
create(c_array<reference_counted_ptr<FontFreeType> > fonts,
       const char *filename, const RenderParams &render_params,
       reference_counted_ptr<FreetypeLib> lib)
{
  if(!lib)
    {
      lib = FASTUIDRAWnew FreetypeLib();
    }

  FT_Face face(nullptr);
  int error_code;
  unsigned int num(0);

  lib->lock();
  error_code = FT_New_Face(lib->lib(), filename, -1, &face);
  lib->unlock();

  if(error_code == 0 && face != nullptr && (face->face_flags & FT_FACE_FLAG_SCALABLE) == 0)
    {
      reference_counted_ptr<fastuidraw::FontFreeType> f;

      num = face->num_faces;
      for(unsigned int i = 0, c = 0; i < num && c < fonts.size(); ++i, ++c)
        {
          reference_counted_ptr<FreetypeFace::GeneratorBase> gen;
          gen = FASTUIDRAWnew FreetypeFace::GeneratorFile(filename, i);
          fonts[c] = FASTUIDRAWnew FontFreeType(gen, render_params, lib);
        }
    }

  if(face != nullptr)
    {
      lib->lock();
      FT_Done_Face(face);
      lib->unlock();
    }

  return num;
}

fastuidraw::FontProperties
fastuidraw::FontFreeType::
compute_font_properties_from_face(FT_Face in_face)
{
  FontProperties return_value;
  compute_font_properties_from_face(in_face, return_value);
  return return_value;
}

void
fastuidraw::FontFreeType::
compute_font_properties_from_face(FT_Face in_face, FontProperties &out_properties)
{
  out_properties.family(in_face->family_name);
  out_properties.style(in_face->style_name);
  out_properties.bold(in_face->style_flags & FT_STYLE_FLAG_BOLD);
  out_properties.italic(in_face->style_flags & FT_STYLE_FLAG_ITALIC);
}
