/*!
 * \file painter_brush_shader_glsl.hpp
 * \brief file painter_brush_shader_glsl.hpp
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

#include <fastuidraw/painter/painter_brush_shader.hpp>
#include <fastuidraw/glsl/shader_source.hpp>

namespace fastuidraw
{
  namespace glsl
  {
/*!\addtogroup GLSLShaderBuilder
  @{
 */
    /*!
      A PainterBrushShaderGLSL is a PainterBrushShader whose
      shader code fragment is via GLSL. A PainterBrushShaderGLSL
      provides a GLSL source code to implement the function
      \code
        void
        fastuidraw_gl_brush_main(in uint sub_shader,
                                 inout uint shader_data_offset,
                                 inout vec2 brush_location,
                                 inout vec4 color);
      \endcode
      where:
       - sub_shader is the sub-shader ID,
       - shader_data_offset is the location of the shader data this
         value must be incremented to the location of the end of
         the brush shader data,
       - brush_location is the position in the brush, this value
         can be written to as well to create more
         effects and
       - color is the color to emit and is initalized as the value
         from the previous brush stage.
     */
    class PainterBrushShaderGLSL:public PainterBrushShader
    {
    public:
      /*!
        Ctor.
        \param src GLSL code fragment for blend shading
        \param num_sub_shaders the number of sub-shaders it supports
       */
      PainterBrushShaderGLSL(const ShaderSource &src, unsigned int num_sub_shaders = 1);

      ~PainterBrushShaderGLSL(void);

      /*!
        Return the GLSL source of the blend shader
       */
      const glsl::ShaderSource&
      brush_src(void) const;

    private:
      void *m_d;
    };
/*! @} */

  }
}
