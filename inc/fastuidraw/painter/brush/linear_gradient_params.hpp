/*!
 * \file linear_gradient_params.hpp
 * \brief file linear_gradient_params.hpp
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


#pragma once

#include <fastuidraw/util/vecN.hpp>
#include <fastuidraw/colorstop_atlas.hpp>
#include <fastuidraw/painter/brush/painter_brush_shader_data.hpp>

namespace fastuidraw
{
/*!\addtogroup Painter
  @{
 */

  /*!
    Class to specify linear gradient parameters, data is packed
    as according to LinearGradientParams::data_offset_t.
   */
  class LinearGradientParams:public PainterBrushShaderData
  {
  public:
    /*!
      Meaning of the bits for flags of the gradient
     */
    enum bit_encoding_t
      {
        /*!
          If bit is up, gradient repeats otherwise gradient is clamped.
         */
        repeat_gradient_bit,
      };

    /*!
      Bit masks derived from \ref bit_encoding_t
     */
    enum bit_masks_t
      {
        /*!
          Mask derived from \ref repeat_gradient_bit
         */
        repeat_gradient_mask = FASTUIDRAW_MASK(repeat_gradient_bit, 1)
      };

    /*!
      Enumeration that provides offset for packing of
      data of a linear gradient.
     */
    enum data_offset_t
      {
        /*!
          Offset to x-coordinate of starting point of gradient,
          packed as a float.
         */
        start_pt_x_offset,

        /*!
          Offset to y-coordinate of starting point of gradient,
          packed as a float.
         */
        start_pt_y_offset,

        /*!
          Offset to x-coordinate of ending point of gradient,
          packed as a float.
         */
        end_pt_x_offset,

        /*!
          Offset to y-coordinate of ending point of gradient,
          packed as a float.
         */
        end_pt_y_offset,

        /*!
          Offset to the value ColorStopSequenceOnAtlas::texel_location().x()
          times the reciprocal of ColorStopBackingStore::dimensions().x()
          for the ColorStopSequenceOnAtlas used by the linear gradient,
          packed as a float.
         */
        color_stop_sequence_x_offset,

        /*!
          Offset to the value ColorStopSequenceOnAtlas::texel_location().y()
          for the ColorStopSequenceOnAtlas used by the linear gradient,
          packed as a float.
         */
        color_stop_sequence_y_offset,

        /*!
          Offset to the value ColorStopSequenceOnAtlas::width()
          times the reciprocal of ColorStopBackingStore::dimensions().x()
          for the ColorStopSequenceOnAtlas used by the linear gradient,
          packed as a float.
         */
        color_stop_sequence_width_offset,

        /*!
          Offset to flags see \ref bit_encoding_t and \ref bit_masks_t
         */
        flags_offset,

        /*!
          Size of the data
         */
        data_size
      };

    /*!
      Ctor.
     */
    LinearGradientParams(void);

    /*!
      Set the value for the start of the gradient
      \param pt value to use for the start point
     */
    LinearGradientParams&
    start_pt(const vec2 &pt);

    /*!
      Return the value as set by start_pt(const vec2&).
     */
    const vec2&
    start_pt(void) const;

    /*!
      Set the value for the end of the gradient
      \param pt value to use for the end point
     */
    LinearGradientParams&
    end_pt(const vec2 &pt);

    /*!
      Return the value as set by end_pt(const vec2&).
     */
    const vec2&
    end_pt(void) const;

    /*!
      Set the value for flags of the gradient
      \param v value to use for the flags
     */
    LinearGradientParams&
    flags(uint32_t v);

    /*!
      Return the value as set by flags(uint32_t).
     */
    uint32_t
    flags(void) const;

    /*!
      Set what color stop sequence to use for the gradient.
      \param cs color stop sequence to use
     */
    LinearGradientParams&
    color_stop_sequence(const reference_counted_ptr<const ColorStopSequenceOnAtlas> &cs);

    /*!
      Returns what color stop sequence to use for the gradient.
     */
    const reference_counted_ptr<const ColorStopSequenceOnAtlas>&
    color_stop_sequence(void) const;
  };

/*! @} */

} //namespace fastuidraw
