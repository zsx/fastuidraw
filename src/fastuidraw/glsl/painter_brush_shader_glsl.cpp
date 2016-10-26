/*!
 * \file painter_brush_shader_glsl.cpp
 * \brief file painter_brush_shader_glsl.cpp
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

#include <fastuidraw/glsl/painter_brush_shader_glsl.hpp>

namespace
{
  class PainterBrushShaderGLSLPrivate
  {
  public:
    explicit
    PainterBrushShaderGLSLPrivate(const fastuidraw::glsl::ShaderSource &src):
      m_src(src)
    {}

    fastuidraw::glsl::ShaderSource m_src;
  };
}


///////////////////////////////////////////////
// fastuidraw::glsl::PainterBrushShaderGLSL methods
fastuidraw::glsl::PainterBrushShaderGLSL::
PainterBrushShaderGLSL(const ShaderSource &src,
                       unsigned int num_sub_shaders):
  PainterBrushShader(num_sub_shaders)
{
  m_d = FASTUIDRAWnew PainterBrushShaderGLSLPrivate(src);
}

fastuidraw::glsl::PainterBrushShaderGLSL::
~PainterBrushShaderGLSL(void)
{
  PainterBrushShaderGLSLPrivate *d;
  d = reinterpret_cast<PainterBrushShaderGLSLPrivate*>(m_d);
  FASTUIDRAWdelete(d);
  m_d = NULL;
}

const fastuidraw::glsl::ShaderSource&
fastuidraw::glsl::PainterBrushShaderGLSL::
brush_src(void) const
{
  PainterBrushShaderGLSLPrivate *d;
  d = reinterpret_cast<PainterBrushShaderGLSLPrivate*>(m_d);
  return d->m_src;
}
