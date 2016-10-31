/*!
 * \file transformation_translation_params.hpp
 * \brief file transformation_translation_params.hpp
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
#include <fastuidraw/painter/brush/painter_brush_shader_data.hpp>

namespace fastuidraw
{
/*!\addtogroup Painter
  @{
 */

  /*!
    Class to specify a translation acting on a brush, data is
    packed as according to TransformationTranslationParams::data_offset_t.
   */
  class TransformationTranslationParams:public PainterBrushShaderData
  {
  public:
    /*!
      Enumeration that provides offset for packing of
      data of a transformation translation.
     */
    enum data_offset_t
      {
        x_offset, /*!< offset for the x-position of the translation (packed at float) */
        y_offset, /*!< offset for the y-position of the translation (packed at float) */
        data_size /*!< size of data */
      };

    /*!
      Ctor.
     */
    TransformationTranslationParams(void);

    /*!
      Gives the about by which to translation the brush.
     */
    const vec2&
    pos(void) const;

    /*!
      Set the value returned by pos(void) const.
     */
    TransformationTranslationParams&
    pos(const vec2 &v);
  };

/*! @} */

} //namespace fastuidraw
