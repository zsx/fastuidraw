/*!
 * \file unified_brush_params.cpp
 * \brief file unified_brush_params.cpp
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

#include <fastuidraw/util/fastuidraw_memory.hpp>
#include <fastuidraw/painter/brush/unified_brush_params.hpp>

namespace
{
  void
  data_size_helper(unsigned int *v, unsigned int alignment,
                   const fastuidraw::PainterBrushShaderData &data,
                   uint16_t shader_flags,
                   enum fastuidraw::UnifiedBrushParams::active_masks_t mask)
  {
    if(mask & shader_flags)
      {
        *v += data.data_size(alignment);
      }
  }

  void
  pack_data_helper(unsigned int *current, fastuidraw::c_array<fastuidraw::generic_data> dst,
                   unsigned int alignment, const fastuidraw::PainterBrushShaderData &data,
                   uint16_t shader_flags,
                   enum fastuidraw::UnifiedBrushParams::active_masks_t mask)
  {
    if(mask & shader_flags)
      {
        unsigned int sz;
        fastuidraw::c_array<fastuidraw::generic_data> sub_dst;

        sz = data.data_size(alignment);
        sub_dst = dst.sub_array(*current, sz);
        data.pack_data(alignment, sub_dst);
        *current += sz;
      }
  }

  class UParams:public fastuidraw::PainterBrushShaderData::DataBase
  {
  public:
    UParams(void):
      m_shader_flags(0u)
    {}

    virtual
    fastuidraw::PainterBrushShaderData::DataBase*
    copy(void) const
    {
      return FASTUIDRAWnew UParams(*this);
    }

    virtual
    fastuidraw::const_c_array<ImageRef>
    images(void) const;

    virtual
    fastuidraw::const_c_array<ColorStopSequenceOnAtlasRef>
    color_stops(void) const;

    virtual
    unsigned int
    data_size(unsigned int alignment) const;

    virtual
    void
    pack_data(unsigned int alignment, fastuidraw::c_array<fastuidraw::generic_data> dst) const;

    virtual
    uint16_t
    shader_flags(void) const
    {
      return m_shader_flags;
    }

    fastuidraw::PenParams m_pen;
    fastuidraw::ImageParams m_image;
    fastuidraw::LinearGradientParams m_linear_gradient;
    fastuidraw::RadialGradientParams m_radial_gradient;
    fastuidraw::RepeatWindowParams m_repeat_window;
    fastuidraw::TransformationTranslationParams m_translation;
    fastuidraw::TransformationMatrixParams m_matrix;
    uint16_t m_shader_flags;
  };
}

///////////////////////////////////////////
// UParams methods
fastuidraw::const_c_array<UParams::ImageRef>
UParams::
images(void) const
{
  if(m_shader_flags & fastuidraw::UnifiedBrushParams::image_mask)
    {
      return m_image.images();
    }
  else
    {
      return fastuidraw::const_c_array<UParams::ImageRef>();
    }
}

fastuidraw::const_c_array<UParams::ColorStopSequenceOnAtlasRef>
UParams::
color_stops(void) const
{
  if(m_shader_flags & fastuidraw::UnifiedBrushParams::linear_gradient_mask)
    {
      return m_linear_gradient.color_stops();
    }
  else if(m_shader_flags & fastuidraw::UnifiedBrushParams::radial_gradient_mask)
    {
      return m_radial_gradient.color_stops();
    }
  else
    {
      return fastuidraw::const_c_array<UParams::ColorStopSequenceOnAtlasRef>();
    }
}

unsigned int
UParams::
data_size(unsigned int alignment) const
{
  unsigned int return_value(0u);

  data_size_helper(&return_value, alignment, m_pen, m_shader_flags, fastuidraw::UnifiedBrushParams::pen_mask);
  data_size_helper(&return_value, alignment, m_matrix, m_shader_flags, fastuidraw::UnifiedBrushParams::transformation_matrix_mask);
  data_size_helper(&return_value, alignment, m_translation, m_shader_flags, fastuidraw::UnifiedBrushParams::transformation_translation_mask);
  data_size_helper(&return_value, alignment, m_repeat_window, m_shader_flags, fastuidraw::UnifiedBrushParams::repeat_window_mask);
  data_size_helper(&return_value, alignment, m_image, m_shader_flags, fastuidraw::UnifiedBrushParams::image_mask);
  data_size_helper(&return_value, alignment, m_linear_gradient, m_shader_flags, fastuidraw::UnifiedBrushParams::linear_gradient_mask);
  data_size_helper(&return_value, alignment, m_radial_gradient, m_shader_flags, fastuidraw::UnifiedBrushParams::radial_gradient_mask);

  return return_value;
}

void
UParams::
pack_data(unsigned int alignment, fastuidraw::c_array<fastuidraw::generic_data> dst) const
{
  unsigned int current(0u);

  pack_data_helper(&current, dst, alignment, m_pen, m_shader_flags, fastuidraw::UnifiedBrushParams::pen_mask);
  pack_data_helper(&current, dst, alignment, m_matrix, m_shader_flags, fastuidraw::UnifiedBrushParams::transformation_matrix_mask);
  pack_data_helper(&current, dst, alignment, m_translation, m_shader_flags, fastuidraw::UnifiedBrushParams::transformation_translation_mask);
  pack_data_helper(&current, dst, alignment, m_repeat_window, m_shader_flags, fastuidraw::UnifiedBrushParams::repeat_window_mask);
  pack_data_helper(&current, dst, alignment, m_image, m_shader_flags, fastuidraw::UnifiedBrushParams::image_mask);
  pack_data_helper(&current, dst, alignment, m_linear_gradient, m_shader_flags, fastuidraw::UnifiedBrushParams::linear_gradient_mask);
  pack_data_helper(&current, dst, alignment, m_radial_gradient, m_shader_flags, fastuidraw::UnifiedBrushParams::radial_gradient_mask);

  assert(current == dst.size());
}

///////////////////////////////////////////
// fastuidraw::UnifiedBrushParams methods
fastuidraw::UnifiedBrushParams::
UnifiedBrushParams(void)
{
  m_data = FASTUIDRAWnew UParams();
}

fastuidraw::UnifiedBrushParams&
fastuidraw::UnifiedBrushParams::
reset(void)
{
  UParams *d;
  d = static_cast<UParams*>(m_data);
  d->m_shader_flags = 0u;
  return *this;
}

fastuidraw::UnifiedBrushParams&
fastuidraw::UnifiedBrushParams::
pen(const vec4 &v)
{
  UParams *d;
  d = static_cast<UParams*>(m_data);
  if(v != vec4(1.0f, 1.0f, 1.0f, 1.0f))
    {
      d->m_shader_flags |= pen_mask;
      d->m_pen.color(v);
    }
  else
    {
      d->m_shader_flags &= ~pen_mask;
    }
  return *this;
}

fastuidraw::UnifiedBrushParams&
fastuidraw::UnifiedBrushParams::
image(const reference_counted_ptr<const Image> &im,
      enum ImageParams::filter_t f)
{
  UParams *d;
  d = static_cast<UParams*>(m_data);
  if(im)
    {
      d->m_shader_flags |= image_mask;
      d->m_image.image(im, f);
    }
  else
    {
      d->m_shader_flags &= ~image_mask;
    }
  return *this;
}

fastuidraw::UnifiedBrushParams&
fastuidraw::UnifiedBrushParams::
sub_image(const reference_counted_ptr<const Image> &im, uvec2 xy, uvec2 wh,
          enum ImageParams::filter_t f)
{
  UParams *d;
  d = static_cast<UParams*>(m_data);
  if(im)
    {
      d->m_shader_flags |= image_mask;
      d->m_image.sub_image(im, xy, wh, f);
    }
  else
    {
      d->m_shader_flags &= ~image_mask;
    }
  return *this;
}

fastuidraw::UnifiedBrushParams&
fastuidraw::UnifiedBrushParams::
no_image(void)
{
  UParams *d;
  d = static_cast<UParams*>(m_data);
  d->m_shader_flags &= ~image_mask;
  return *this;
}

fastuidraw::UnifiedBrushParams&
fastuidraw::UnifiedBrushParams::
linear_gradient(const reference_counted_ptr<const ColorStopSequenceOnAtlas> &cs,
                const vec2 &start_p, const vec2 &end_p, bool repeat)
{
  UParams *d;
  d = static_cast<UParams*>(m_data);
  d->m_shader_flags &= ~radial_gradient_mask;
  if(cs)
    {
      uint32_t flags;
      flags = repeat ? uint32_t(LinearGradientParams::repeat_gradient_mask) : 0u;

      d->m_linear_gradient
        .start_pt(start_p)
        .end_pt(end_p)
        .color_stop_sequence(cs)
        .flags(flags);
      d->m_shader_flags |= linear_gradient_mask;
    }
  else
    {
      d->m_shader_flags &= ~linear_gradient_mask;
    }
  return *this;
}

fastuidraw::UnifiedBrushParams&
fastuidraw::UnifiedBrushParams::
radial_gradient(const reference_counted_ptr<const ColorStopSequenceOnAtlas> &cs,
                const vec2 &start_p, float start_r,
                const vec2 &end_p, float end_r, bool repeat)
{
  UParams *d;
  d = static_cast<UParams*>(m_data);
  d->m_shader_flags &= ~linear_gradient_mask;
  if(cs)
    {
      uint32_t flags;
      flags = repeat ? uint32_t(RadialGradientParams::repeat_gradient_mask) : 0u;

      d->m_radial_gradient
        .start_pt(start_p)
        .start_r(start_r)
        .end_pt(end_p)
        .end_r(end_r)
        .color_stop_sequence(cs)
        .flags(flags);

      d->m_shader_flags |= radial_gradient_mask;
    }
  else
    {
      d->m_shader_flags &= ~radial_gradient_mask;
    }
  return *this;
}

fastuidraw::UnifiedBrushParams&
fastuidraw::UnifiedBrushParams::
no_gradient(void)
{
  UParams *d;
  d = static_cast<UParams*>(m_data);
  d->m_shader_flags &= ~(linear_gradient_mask | radial_gradient_mask);
  return *this;
}


fastuidraw::UnifiedBrushParams&
fastuidraw::UnifiedBrushParams::
transformation_translate(const vec2 &p)
{
  UParams *d;
  d = static_cast<UParams*>(m_data);
  d->m_shader_flags |= transformation_translation_mask;
  d->m_translation.pos(p);
  return *this;
}

fastuidraw::UnifiedBrushParams&
fastuidraw::UnifiedBrushParams::
transformation_matrix(const float2x2 &p)
{
  UParams *d;
  d = static_cast<UParams*>(m_data);
  d->m_shader_flags |= transformation_matrix_mask;
  d->m_matrix.matrix(p);
  return *this;
}

fastuidraw::UnifiedBrushParams&
fastuidraw::UnifiedBrushParams::
no_transformation_translation(void)
{
  UParams *d;
  d = static_cast<UParams*>(m_data);
  d->m_shader_flags &= ~transformation_translation_mask;
  return *this;
}

fastuidraw::UnifiedBrushParams&
fastuidraw::UnifiedBrushParams::
no_transformation_matrix(void)
{
  UParams *d;
  d = static_cast<UParams*>(m_data);
  d->m_shader_flags &= ~transformation_matrix_mask;
  return *this;
}


fastuidraw::UnifiedBrushParams&
fastuidraw::UnifiedBrushParams::
repeat_window(const vec2 &pos, const vec2 &size)
{
  UParams *d;
  d = static_cast<UParams*>(m_data);
  d->m_shader_flags |= repeat_window_mask;
  d->m_repeat_window.pos(pos).size(size);
  return *this;
}

fastuidraw::UnifiedBrushParams&
fastuidraw::UnifiedBrushParams::
no_repeat_window(void)
{
  UParams *d;
  d = static_cast<UParams*>(m_data);
  d->m_shader_flags &= ~repeat_window_mask;
  return *this;
}
