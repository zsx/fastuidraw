/*!
 * \file painter_brush.cpp
 * \brief file painter_brush.cpp
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
#include <algorithm>
#include <fastuidraw/painter/brush/painter_brush.hpp>

namespace
{
  void
  data_size_helper(unsigned int *v, unsigned int alignment,
                   const fastuidraw::PainterBrushShaderData &data,
                   uint32_t shader, uint32_t mask)
  {
    if(mask & shader)
      {
        *v += data.data_size(alignment);
      }
  }

  void
  pack_data_helper(unsigned int *current, fastuidraw::c_array<fastuidraw::generic_data> dst,
                   unsigned int alignment, const fastuidraw::PainterBrushShaderData &data,
                   uint32_t shader, uint32_t mask)
  {
    if(mask & shader)
      {
        unsigned int sz;
        fastuidraw::c_array<fastuidraw::generic_data> sub_dst;

        sz = data.data_size(alignment);
        sub_dst = dst.sub_array(*current, sz);
        data.pack_data(alignment, sub_dst);
        *current += sz;
      }
  }

  class DerivedValuePrivate
  {
  public:
    typedef fastuidraw::Image Image;
    typedef fastuidraw::reference_counted_ptr<const Image> ImageRef;
    typedef fastuidraw::ColorStopSequenceOnAtlas ColorStops;
    typedef fastuidraw::reference_counted_ptr<const ColorStops> ColorStopsRef;

    DerivedValuePrivate(void):
      m_shader(0u),
      m_images(1),
      m_color_stops(1)
    {}

    uint32_t m_shader;
    std::vector<ImageRef> m_images;
    std::vector<ColorStopsRef> m_color_stops;
  };
}

////////////////////////////////////
// derived_value_type methods
fastuidraw::PainterBrush::derived_value_type::
derived_value_type(void)
{
  m_d = FASTUIDRAWnew DerivedValuePrivate();
}

fastuidraw::PainterBrush::derived_value_type::
derived_value_type(const derived_value_type &obj)
{
  m_d = FASTUIDRAWnew DerivedValuePrivate(*reinterpret_cast<DerivedValuePrivate*>(obj.m_d));
}

fastuidraw::PainterBrush::derived_value_type::
~derived_value_type()
{
  DerivedValuePrivate *d;
  d = reinterpret_cast<DerivedValuePrivate*>(m_d);
  FASTUIDRAWdelete(d);
}

fastuidraw::PainterBrush::derived_value_type&
fastuidraw::PainterBrush::derived_value_type::
operator=(const derived_value_type &rhs)
{
  if(this != &rhs)
    {
      *reinterpret_cast<DerivedValuePrivate*>(m_d) = *reinterpret_cast<DerivedValuePrivate*>(rhs.m_d);
    }
  return *this;
}

uint32_t
fastuidraw::PainterBrush::derived_value_type::
shader(void) const
{
  DerivedValuePrivate *d;
  d = reinterpret_cast<DerivedValuePrivate*>(m_d);
  return d->m_shader;
}

const fastuidraw::reference_counted_ptr<const fastuidraw::Image>&
fastuidraw::PainterBrush::derived_value_type::
image(void) const
{
  DerivedValuePrivate *d;
  d = reinterpret_cast<DerivedValuePrivate*>(m_d);
  return d->m_images[0];
}

const fastuidraw::reference_counted_ptr<const fastuidraw::ColorStopSequenceOnAtlas>&
fastuidraw::PainterBrush::derived_value_type::
colorstops(void) const
{
  DerivedValuePrivate *d;
  d = reinterpret_cast<DerivedValuePrivate*>(m_d);
  return d->m_color_stops[0];
}

////////////////////////////////////
// fastuidraw::PainterBrush methods
unsigned int
fastuidraw::PainterBrush::
data_size(unsigned int alignment) const
{
  unsigned int return_value(0u);
  uint32_t shader_value(shader());

  data_size_helper(&return_value, alignment, m_pen, 1, 1);
  data_size_helper(&return_value, alignment, m_image, shader_value, image_mask);
  data_size_helper(&return_value, alignment, m_linear_gradient, shader_value, linear_gradient_mask);
  data_size_helper(&return_value, alignment, m_radial_gradient, shader_value, radial_gradient_mask);
  data_size_helper(&return_value, alignment, m_repeat_window, shader_value, repeat_window_mask);
  data_size_helper(&return_value, alignment, m_translation, shader_value, transformation_translation_mask);
  data_size_helper(&return_value, alignment, m_matrix, shader_value, transformation_matrix_mask);

  return return_value;
}

void
fastuidraw::PainterBrush::
pack_data(unsigned int alignment, c_array<generic_data> dst) const
{
  unsigned int current(0);
  uint32_t shader_value(shader());

  pack_data_helper(&current, dst, alignment, m_pen, 1, 1);
  pack_data_helper(&current, dst, alignment, m_image, shader_value, image_mask);
  pack_data_helper(&current, dst, alignment, m_linear_gradient, shader_value, linear_gradient_mask);
  pack_data_helper(&current, dst, alignment, m_radial_gradient, shader_value, radial_gradient_mask);
  pack_data_helper(&current, dst, alignment, m_repeat_window, shader_value, repeat_window_mask);
  pack_data_helper(&current, dst, alignment, m_translation, shader_value, transformation_translation_mask);
  pack_data_helper(&current, dst, alignment, m_matrix, shader_value, transformation_matrix_mask);

  assert(current == dst.size());
}

fastuidraw::PainterBrush&
fastuidraw::PainterBrush::
reset(void)
{
  DerivedValuePrivate *d;
  d = reinterpret_cast<DerivedValuePrivate*>(m_derived_value.m_d);
  d->m_shader = 0u;
  return pen(1.0f, 1.0f, 1.0f, 1.0f);
}

uint32_t
fastuidraw::PainterBrush::
shader(void) const
{
  return derived_value().shader();
}

fastuidraw::PainterBrush&
fastuidraw::PainterBrush::
image(const reference_counted_ptr<const Image> &im, enum ImageParams::filter_t f)
{
  DerivedValuePrivate *d;
  d = reinterpret_cast<DerivedValuePrivate*>(m_derived_value.m_d);
  m_image.image(im, f);
  d->m_shader = apply_bit_flag(d->m_shader, im, image_mask);
  d->m_images[0] = im;
  return *this;
}

fastuidraw::PainterBrush&
fastuidraw::PainterBrush::
sub_image(const reference_counted_ptr<const Image> &im, uvec2 xy, uvec2 wh,
          enum ImageParams::filter_t f)
{
  DerivedValuePrivate *d;
  d = reinterpret_cast<DerivedValuePrivate*>(m_derived_value.m_d);
  m_image.sub_image(im, xy, wh, f);
  d->m_shader = apply_bit_flag(d->m_shader, im, image_mask);
  d->m_images[0] = im;
  return *this;
}

fastuidraw::PainterBrush&
fastuidraw::PainterBrush::
linear_gradient(const reference_counted_ptr<const ColorStopSequenceOnAtlas> &cs,
                const vec2 &start_p, const vec2 &end_p, bool repeat)
{
  DerivedValuePrivate *d;
  d = reinterpret_cast<DerivedValuePrivate*>(m_derived_value.m_d);
  uint32_t flags;
  flags = repeat ? uint32_t(LinearGradientParams::repeat_gradient_mask) : 0u;
  m_linear_gradient
    .start_pt(start_p)
    .end_pt(end_p)
    .color_stop_sequence(cs)
    .flags(flags);
  d->m_shader = apply_bit_flag(d->m_shader, cs, linear_gradient_mask);
  d->m_shader = apply_bit_flag(d->m_shader, false, radial_gradient_mask);
  d->m_shader = apply_bit_flag(d->m_shader, cs && repeat, repeat_gradient_mask);
  d->m_color_stops[0] = cs;
  return *this;
}

fastuidraw::PainterBrush&
fastuidraw::PainterBrush::
radial_gradient(const reference_counted_ptr<const ColorStopSequenceOnAtlas> &cs,
                const vec2 &start_p, float start_r,
                const vec2 &end_p, float end_r, bool repeat)
{
  DerivedValuePrivate *d;
  d = reinterpret_cast<DerivedValuePrivate*>(m_derived_value.m_d);
  uint32_t flags;
  flags = repeat ? uint32_t(RadialGradientParams::repeat_gradient_mask) : 0u;
  m_radial_gradient
    .start_pt(start_p)
    .start_r(start_r)
    .end_pt(end_p)
    .end_r(end_r)
        .color_stop_sequence(cs)
    .flags(flags);
  d->m_shader = apply_bit_flag(d->m_shader, cs, radial_gradient_mask);
  d->m_shader = apply_bit_flag(d->m_shader, false, linear_gradient_mask);
  d->m_shader = apply_bit_flag(d->m_shader, cs && repeat, repeat_gradient_mask);
  d->m_color_stops[0] = cs;
  return *this;
}

fastuidraw::PainterBrush&
fastuidraw::PainterBrush::
no_gradient(void)
{
  DerivedValuePrivate *d;
  d = reinterpret_cast<DerivedValuePrivate*>(m_derived_value.m_d);
  d->m_shader &= ~(linear_gradient_mask | radial_gradient_mask | repeat_gradient_mask);
  return *this;
}

fastuidraw::PainterBrush&
fastuidraw::PainterBrush::
transformation_translate(const vec2 &p)
{
  DerivedValuePrivate *d;
  d = reinterpret_cast<DerivedValuePrivate*>(m_derived_value.m_d);
  m_translation.pos(p);
  d->m_shader |= transformation_translation_mask;
  return *this;
}

fastuidraw::PainterBrush&
fastuidraw::PainterBrush::
transformation_matrix(const float2x2 &m)
{
  DerivedValuePrivate *d;
  d = reinterpret_cast<DerivedValuePrivate*>(m_derived_value.m_d);
  m_matrix.matrix(m);
  d->m_shader |= transformation_matrix_mask;
  return *this;
}

fastuidraw::PainterBrush&
fastuidraw::PainterBrush::
no_transformation_translation(void)
{
  DerivedValuePrivate *d;
  d = reinterpret_cast<DerivedValuePrivate*>(m_derived_value.m_d);
  d->m_shader &= ~transformation_translation_mask;
  return *this;
}

fastuidraw::PainterBrush&
fastuidraw::PainterBrush::
no_transformation_matrix(void)
{
  DerivedValuePrivate *d;
  d = reinterpret_cast<DerivedValuePrivate*>(m_derived_value.m_d);
  d->m_shader &= ~transformation_matrix_mask;
  return *this;
}

fastuidraw::PainterBrush&
fastuidraw::PainterBrush::
repeat_window(const vec2 &pos, const vec2 &size)
{
  DerivedValuePrivate *d;
  d = reinterpret_cast<DerivedValuePrivate*>(m_derived_value.m_d);
  m_repeat_window
    .pos(pos)
    .size(size);
  d->m_shader |= repeat_window_mask;
  return *this;
}

fastuidraw::PainterBrush&
fastuidraw::PainterBrush::
no_repeat_window(void)
{
  DerivedValuePrivate *d;
  d = reinterpret_cast<DerivedValuePrivate*>(m_derived_value.m_d);
  d->m_shader &= ~repeat_window_mask;
  return *this;
}
