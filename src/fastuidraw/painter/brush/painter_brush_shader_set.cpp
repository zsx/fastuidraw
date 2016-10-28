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

#include <vector>
#include <fastuidraw/painter/brush/painter_brush_shader_set.hpp>

namespace
{
  class PainterBrushShaderSetPrivate
  {
  public:
    std::vector<fastuidraw::reference_counted_ptr<fastuidraw::PainterBrushShader> > m_shaders;
    fastuidraw::reference_counted_ptr<fastuidraw::PainterBrushShader> m_null;
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

fastuidraw::PainterBrushShaderSet&
fastuidraw::PainterBrushShaderSet::
shader(enum effect_t tp, const reference_counted_ptr<PainterBrushShader> &v)
{
  PainterBrushShaderSetPrivate *d;
  d = reinterpret_cast<PainterBrushShaderSetPrivate*>(m_d);
  if(tp >= d->m_shaders.size())
    {
      d->m_shaders.resize(tp + 1);
    }
  d->m_shaders[tp] = v;
  return *this;
}

const fastuidraw::reference_counted_ptr<fastuidraw::PainterBrushShader>&
fastuidraw::PainterBrushShaderSet::
shader(enum effect_t tp) const
{
  PainterBrushShaderSetPrivate *d;
  d = reinterpret_cast<PainterBrushShaderSetPrivate*>(m_d);
  return (tp < d->m_shaders.size()) ?
    d->m_shaders[tp] :
    d->m_null;
}

unsigned int
fastuidraw::PainterBrushShaderSet::
shader_count(void) const
{
  PainterBrushShaderSetPrivate *d;
  d = reinterpret_cast<PainterBrushShaderSetPrivate*>(m_d);
  return d->m_shaders.size();
}
