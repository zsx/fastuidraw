/*!
 * \file painter_brush_shader.hpp
 * \brief file painter_brush_shader.hpp
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
#include <fastuidraw/painter/painter_shader.hpp>

namespace fastuidraw
{

/*!\addtogroup Painter
  @{
 */

  /*!
    A PainterBrushShader represents a shader to
    apply a brush effect (for example a gradient
    or image).
   */
  class PainterBrushShader:public PainterShader
  {
  public:
    /*!
      Ctor for a PainterBrushShader with no sub-shaders.
     */
    PainterBrushShader(void):
      PainterShader()
    {}

    /*!
      Ctor for creating a PainterBrushShader which has multiple
      sub-shaders. The purpose of sub-shaders is for the
      case where multiple shaders almost same code and those
      code differences can be realized by examining a sub-shader
      ID.
      \param num_sub_shaders number of sub-shaders
     */
    explicit
    PainterBrushShader(unsigned int num_sub_shaders):
      PainterShader(num_sub_shaders)
    {}

    /*!
      Ctor to create a PainterBrushShader realized as a sub-shader
      of an existing PainterBrushShader
      \param sub_shader which sub-shader of the parent PainterBrushShader
      \param parent parent PainterBrushShader that has sub-shaders
     */
    PainterBrushShader(unsigned int sub_shader,
                      reference_counted_ptr<PainterBrushShader> parent):
      PainterShader(sub_shader, parent)
    {}
  };

/*! @} */
}
