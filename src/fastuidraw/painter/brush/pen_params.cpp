/*!
 * \file pen_params.cpp
 * \brief file pen_params.cpp
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

#include <fastuidraw/util/fastuidraw_memory.hpp>
#include <fastuidraw/painter/brush/pen_params.hpp>

namespace
{
  class PParams:public fastuidraw::PainterBrushShaderData::DataBase
  {
  public:
    PParams(void):
      m_color(1.0f, 1.0f, 1.0f, 1.0f)
    {}

    virtual
    fastuidraw::PainterBrushShaderData::DataBase*
    copy(void) const
    {
      return FASTUIDRAWnew PParams(*this);
    }

    virtual
    unsigned int
    data_size(unsigned int alignment) const
    {
      return fastuidraw::round_up_to_multiple(fastuidraw::PenParams::data_size, alignment);
    }

    virtual
    void
    pack_data(unsigned int alignment, fastuidraw::c_array<fastuidraw::generic_data> dst) const
    {
      FASTUIDRAWunused(alignment);
      dst[fastuidraw::PenParams::red_offset].f = m_color[0];
      dst[fastuidraw::PenParams::green_offset].f = m_color[1];
      dst[fastuidraw::PenParams::blue_offset].f = m_color[2];
      dst[fastuidraw::PenParams::alpha_offset].f = m_color[3];
    }

    fastuidraw::vec4 m_color;
  };
}


//////////////////////////////////////
// PenParams methods
fastuidraw::PenParams::
PenParams(void)
{
  m_data = FASTUIDRAWnew PParams();
}

#define set_get_implement(name, type)                                 \
  fastuidraw::PenParams&                                              \
  fastuidraw::PenParams::                                             \
  name(type v)                                                        \
  {                                                                   \
    PParams *d;                                                       \
    assert(dynamic_cast<PParams*>(m_data) != NULL);                   \
    d = static_cast<PParams*>(m_data);                                \
    d->m_##name = v;                                                  \
    return *this;                                                     \
  }                                                                   \
                                                                      \
  type                                                                \
  fastuidraw::PenParams::                                             \
  name(void) const                                                    \
  {                                                                   \
    PParams *d;                                                       \
    assert(dynamic_cast<PParams*>(m_data) != NULL);                   \
    d = static_cast<PParams*>(m_data);                                \
    return d->m_##name;                                               \
  }

set_get_implement(color, const fastuidraw::vec4&)

#undef set_get_implement
