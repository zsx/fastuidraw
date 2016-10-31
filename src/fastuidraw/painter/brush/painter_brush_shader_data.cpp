/*!
 * \file painter_brush_shader_data.cpp
 * \brief file painter_brush_shader_data.cpp
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


#include <fastuidraw/painter/brush/painter_brush_shader_data.hpp>
#include <fastuidraw/util/fastuidraw_memory.hpp>

//////////////////////////////////////////////
// fastuidraw::PainterBrushShaderData methods
fastuidraw::PainterBrushShaderData::
PainterBrushShaderData(void)
{
  m_data = NULL;
}

fastuidraw::PainterBrushShaderData::
PainterBrushShaderData(const PainterBrushShaderData &obj)
{
  if(obj.m_data)
    {
      m_data = obj.m_data->copy();
    }
  else
    {
      m_data = NULL;
    }
}

fastuidraw::PainterBrushShaderData::
~PainterBrushShaderData()
{
  if(m_data)
    {
      FASTUIDRAWdelete(m_data);
    }
  m_data = NULL;
}

fastuidraw::PainterBrushShaderData&
fastuidraw::PainterBrushShaderData::
operator=(const PainterBrushShaderData &rhs)
{
  if(this != &rhs)
    {
      if(m_data)
        {
          FASTUIDRAWdelete(m_data);
        }

      if(rhs.m_data)
        {
          m_data = rhs.m_data->copy();
        }
    }
  return *this;
}

void
fastuidraw::PainterBrushShaderData::
pack_data(unsigned int alignment, c_array<generic_data> dst) const
{
  if(m_data)
    {
      m_data->pack_data(alignment, dst);
    }
}

unsigned int
fastuidraw::PainterBrushShaderData::
data_size(unsigned int alignment) const
{
  return m_data ?
    m_data->data_size(alignment) :
    0;
}
