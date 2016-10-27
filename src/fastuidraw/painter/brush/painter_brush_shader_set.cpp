/*!
 * \file painter_brush_shader_set.cpp
 * \brief file painter_brush_shader_set.cpp
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


#include <fastuidraw/painter/brush/painter_brush_shader_set.hpp>

namespace
{
  class PainterBrushShaderSetPrivate
  {
  public:
    fastuidraw::reference_counted_ptr<fastuidraw::PainterBrushShader> m_const_color;
    fastuidraw::reference_counted_ptr<fastuidraw::PainterBrushShader> m_linear_gradient;
    fastuidraw::reference_counted_ptr<fastuidraw::PainterBrushShader> m_radial_gradient;
    fastuidraw::reference_counted_ptr<fastuidraw::PainterBrushShader> m_image;
    fastuidraw::reference_counted_ptr<fastuidraw::PainterBrushShader> m_repeat_window;
    fastuidraw::reference_counted_ptr<fastuidraw::PainterBrushShader> m_transformation_translation;
    fastuidraw::reference_counted_ptr<fastuidraw::PainterBrushShader> m_transformation_matrix;
  };
}

//////////////////////////////////////////////
// fastuidraw::PainterBrushShaderSet methods
fastuidraw::PainterBrushShaderSet::
PainterBrushShaderSet(void)
{
  m_d = FASTUIDRAWnew PainterBrushShaderSetPrivate();
}

fastuidraw::PainterBrushShaderSet::
PainterBrushShaderSet(const PainterBrushShaderSet &obj)
{
  PainterBrushShaderSetPrivate *d;
  d = reinterpret_cast<PainterBrushShaderSetPrivate*>(obj.m_d);
  m_d = FASTUIDRAWnew PainterBrushShaderSetPrivate(*d);
}

fastuidraw::PainterBrushShaderSet::
~PainterBrushShaderSet()
{
  PainterBrushShaderSetPrivate *d;
  d = reinterpret_cast<PainterBrushShaderSetPrivate*>(m_d);
  FASTUIDRAWdelete(d);
  m_d = NULL;
}

fastuidraw::PainterBrushShaderSet&
fastuidraw::PainterBrushShaderSet::
operator=(const PainterBrushShaderSet &rhs)
{
  if(this != &rhs)
    {
      PainterBrushShaderSetPrivate *d, *rhs_d;
      d = reinterpret_cast<PainterBrushShaderSetPrivate*>(m_d);
      rhs_d = reinterpret_cast<PainterBrushShaderSetPrivate*>(rhs.m_d);
      *d = *rhs_d;
    }
  return *this;
}

#define setget_implement(name)                                          \
  fastuidraw::PainterBrushShaderSet&                                    \
  fastuidraw::PainterBrushShaderSet::                                   \
  name(const fastuidraw::reference_counted_ptr<fastuidraw::PainterBrushShader> &v) \
  {                                                                     \
    PainterBrushShaderSetPrivate *d;                                    \
    d = reinterpret_cast<PainterBrushShaderSetPrivate*>(m_d);           \
    d->m_##name = v;                                                    \
    return *this;                                                       \
  }                                                                     \
                                                                        \
  const fastuidraw::reference_counted_ptr<fastuidraw::PainterBrushShader>& \
  fastuidraw::PainterBrushShaderSet::                                   \
  name(void) const                                                      \
  {                                                                     \
    PainterBrushShaderSetPrivate *d;                                    \
    d = reinterpret_cast<PainterBrushShaderSetPrivate*>(m_d);           \
    return d->m_##name;                                                 \
  }


setget_implement(const_color)
setget_implement(linear_gradient)
setget_implement(radial_gradient)
setget_implement(image)
setget_implement(repeat_window)
setget_implement(transformation_translation)
setget_implement(transformation_matrix)

#undef setget_implement
