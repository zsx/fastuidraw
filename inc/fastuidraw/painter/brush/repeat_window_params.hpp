/*!
 * \file repeat_window_params.hpp
 * \brief file repeat_window_params.hpp
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
#include <fastuidraw/painter/painter_shader_data.hpp>

namespace fastuidraw
{
/*!\addtogroup Painter
  @{
 */

  /*!
    Class to specify a repeat window for a brush, data is
    packed as according to RepeatWindowParams::data_offset_t.
   */
  class RepeatWindowParams:public PainterBrushShaderData
  {
  public:
    /*!
      Enumeration that provides offset from the start of
      repeat window packing to data for repeat window data
     */
    enum data_offset_t
      {
        x_offset, /*!< offset for the x-position of the repeat window (packed at float) */
        y_offset, /*!< offset for the y-position of the repeat window (packed at float) */
        width_offset, /*!< offset for the width of the repeat window (packed at float) */
        height_offset, /*!< offset for the height of the repeat window (packed at float) */
        data_size /*!< size of data for repeat window */
      };

    /*!
      Ctor.
     */
    RepeatWindowParams(void);

    /*!
      Gives the location of the repeat window.
     */
    const vec2&
    pos(void) const;

    /*!
      Set the value returned by pos(void) const.
     */
    RepeatWindowParams&
    pos(const vec2 &v);

    /*!
      Gives the size of the repeat window.
     */
    const vec2&
    size(void) const;

    /*!
      Set the value returned by size(void) const.
     */
    RepeatWindowParams&
    size(const vec2 &v);
  };

/*! @} */

} //namespace fastuidraw
