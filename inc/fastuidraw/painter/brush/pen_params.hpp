/*!
 * \file pen_params.hpp
 * \brief file pen_params.hpp
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
#include <fastuidraw/util/matrix.hpp>
#include <fastuidraw/painter/brush/painter_brush_shader_data.hpp>

namespace fastuidraw
{
/*!\addtogroup Painter
  @{
 */

  /*!
    Class to specify a color for a brush, data is
    packed as according to PenParams::data_offset_t.
   */
  class PenParams:public PainterBrushShaderData
  {
  public:
    /*!
      Enumeration that provides offset for packing of
      data of a pen.
     */
    enum data_offset_t
      {
        red_offset, /*!< offset for pen red value */
        green_offset, /*!< offset for pen green value */
        blue_offset, /*!< offset for pen blue value */
        alpha_offset, /*!< offset for pen alpha value */
        data_size /*!< size of data */
      };

    /*!
      Ctor.
     */
    PenParams(void);

    /*!
      Gives the matrix to apply to the brush.
     */
    const vec4&
    color(void) const;

    /*!
      Set the value returned by color(void) const.
     */
    PenParams&
    color(const vec4 &v);

    /*!
      Set the value returned by color(void) const.
     */
    PenParams&
    color(float r, float g, float b, float a = 1.0f)
    {
      return color(vec4(r, g, b, a));
    }
  };

/*! @} */

} //namespace fastuidraw
