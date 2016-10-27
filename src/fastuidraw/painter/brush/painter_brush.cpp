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

#include <algorithm>
#include <fastuidraw/painter/brush/painter_brush.hpp>

namespace
{
  void
  data_size_helper(unsigned int *v, unsigned int alignment,
                   const fastuidraw::PainterShaderData &data,
                   uint32_t shader, uint32_t mask)
  {
    if(mask & shader)
      {
        *v += data.data_size(alignment);
      }
  }

  void
  pack_data_helper(unsigned int *current, fastuidraw::c_array<fastuidraw::generic_data> dst,
                   unsigned int alignment, const fastuidraw::PainterShaderData &data,
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
}

////////////////////////////////////
// fastuidraw::PainterBrush methods
unsigned int
fastuidraw::PainterBrush::
data_size(unsigned int alignment) const
{
  unsigned int return_value(0u);

  data_size_helper(&return_value, alignment, m_pen, 1, 1);
  data_size_helper(&return_value, alignment, m_image, shader(), image_mask);
  data_size_helper(&return_value, alignment, m_linear_gradient, shader(), linear_gradient_mask);
  data_size_helper(&return_value, alignment, m_radial_gradient, shader(), radial_gradient_mask);
  data_size_helper(&return_value, alignment, m_repeat_window, shader(), repeat_window_mask);
  data_size_helper(&return_value, alignment, m_translation, shader(), transformation_translation_mask);
  data_size_helper(&return_value, alignment, m_matrix, shader(), transformation_matrix_mask);

  return return_value;
}

void
fastuidraw::PainterBrush::
pack_data(unsigned int alignment, c_array<generic_data> dst) const
{
  unsigned int current(0);

  pack_data_helper(&current, dst, alignment, m_pen, 1, 1);
  pack_data_helper(&current, dst, alignment, m_image, shader(), image_mask);
  pack_data_helper(&current, dst, alignment, m_linear_gradient, shader(), linear_gradient_mask);
  pack_data_helper(&current, dst, alignment, m_radial_gradient, shader(), radial_gradient_mask);
  pack_data_helper(&current, dst, alignment, m_repeat_window, shader(), repeat_window_mask);
  pack_data_helper(&current, dst, alignment, m_translation, shader(), transformation_translation_mask);
  pack_data_helper(&current, dst, alignment, m_matrix, shader(), transformation_matrix_mask);

  assert(current == dst.size());
}
