/*!
 * \file repeat_window_params.cpp
 * \brief file repeat_window_params.cpp
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
#include <fastuidraw/painter/brush/repeat_window_params.hpp>
namespace
{
  class RWParams:public fastuidraw::PainterShaderData::DataBase
  {
  public:
    RWParams(void):
      m_pos(0.0f, 0.0f),
      m_size(1.0f, 1.0f)
    {}

    virtual
    fastuidraw::PainterShaderData::DataBase*
    copy(void) const
    {
      return FASTUIDRAWnew RWParams(*this);
    }

    virtual
    unsigned int
    data_size(unsigned int alignment) const
    {
      return fastuidraw::round_up_to_multiple(fastuidraw::RepeatWindowParams::data_size, alignment);
    }

    virtual
    void
    pack_data(unsigned int alignment, fastuidraw::c_array<fastuidraw::generic_data> dst) const
    {
      FASTUIDRAWunused(alignment);
      dst[fastuidraw::RepeatWindowParams::x_offset].f = m_pos.x();
      dst[fastuidraw::RepeatWindowParams::y_offset].f = m_pos.y();
      dst[fastuidraw::RepeatWindowParams::width_offset].f = m_size.x();
      dst[fastuidraw::RepeatWindowParams::height_offset].f = m_size.y();
    }

    fastuidraw::vec2 m_pos, m_size;
  };
}


//////////////////////////////////////
// RepeatWindowParams methods
fastuidraw::RepeatWindowParams::
RepeatWindowParams(void)
{
  m_data = FASTUIDRAWnew RWParams();
}

#define set_get_implement(name, type)                     \
  fastuidraw::RepeatWindowParams&                         \
  fastuidraw::RepeatWindowParams::                        \
  name(type v)                                            \
  {                                                       \
    RWParams *d;                                          \
    assert(dynamic_cast<RWParams*>(m_data) != NULL);      \
    d = static_cast<RWParams*>(m_data);                   \
    d->m_##name = v;                                      \
    return *this;                                         \
  }                                                       \
                                                          \
  type                                                    \
  fastuidraw::RepeatWindowParams::                        \
  name(void) const                                        \
  {                                                       \
    RWParams *d;                                          \
    assert(dynamic_cast<RWParams*>(m_data) != NULL);      \
    d = static_cast<RWParams*>(m_data);                   \
    return d->m_##name;                                   \
  }

set_get_implement(pos, const fastuidraw::vec2&)
set_get_implement(size, const fastuidraw::vec2&)

#undef set_get_implement
