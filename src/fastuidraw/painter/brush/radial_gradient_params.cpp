/*!
 * \file radial_gradient_params.cpp
 * \brief file radial_gradient_params.cpp
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
#include <fastuidraw/painter/brush/radial_gradient_params.hpp>

namespace
{
  class Params:public fastuidraw::PainterShaderData::DataBase
  {
  public:
    Params(void):
      m_start_pt(0.0f, 0.0f),
      m_end_pt(1.0f, 1.0f),
      m_start_r(0.0f),
      m_end_r(1.0f),
      m_flags(0u),
      m_color_stop_sequence()
    {}

    virtual
    fastuidraw::PainterShaderData::DataBase*
    copy(void) const
    {
      return FASTUIDRAWnew Params(*this);
    }

    virtual
    unsigned int
    data_size(unsigned int alignment) const
    {
      return fastuidraw::round_up_to_multiple(fastuidraw::RadialGradientParams::data_size, alignment);
    }

    virtual
    void
    pack_data(unsigned int alignment, fastuidraw::c_array<fastuidraw::generic_data> dst) const
    {
      FASTUIDRAWunused(alignment);
      dst[fastuidraw::RadialGradientParams::start_pt_x_offset].f = m_start_pt.x();
      dst[fastuidraw::RadialGradientParams::start_pt_y_offset].f = m_start_pt.y();
      dst[fastuidraw::RadialGradientParams::start_r_offset].f = m_start_r;
      dst[fastuidraw::RadialGradientParams::end_pt_x_offset].f = m_end_pt.x();
      dst[fastuidraw::RadialGradientParams::end_pt_y_offset].f = m_end_pt.y();
      dst[fastuidraw::RadialGradientParams::end_r_offset].f = m_end_r;
      dst[fastuidraw::RadialGradientParams::flags_offset].u = m_flags;
      if(m_color_stop_sequence)
        {
          int d;
          float r;

          d = m_color_stop_sequence->atlas()->backing_store()->dimensions().x();
          r = 1.0f / static_cast<float>(d);

          dst[fastuidraw::RadialGradientParams::color_stop_sequence_x_offset].f = r * static_cast<float>(m_color_stop_sequence->texel_location().x());
          dst[fastuidraw::RadialGradientParams::color_stop_sequence_y_offset].f = r * static_cast<float>(m_color_stop_sequence->texel_location().y());
          dst[fastuidraw::RadialGradientParams::color_stop_sequence_width_offset].f = r * static_cast<float>(m_color_stop_sequence->width());
        }
      else
        {
          dst[fastuidraw::RadialGradientParams::color_stop_sequence_x_offset].f = 0.0f;
          dst[fastuidraw::RadialGradientParams::color_stop_sequence_y_offset].f = 0.0f;
          dst[fastuidraw::RadialGradientParams::color_stop_sequence_width_offset].f = 0.0f;
        }
    }

    fastuidraw::vec2 m_start_pt, m_end_pt;
    float m_start_r, m_end_r;
    uint32_t m_flags;
    fastuidraw::reference_counted_ptr<const fastuidraw::ColorStopSequenceOnAtlas> m_color_stop_sequence;
  };
}

//////////////////////////////////////
// RadialGradientParams methods
fastuidraw::RadialGradientParams::
RadialGradientParams(void)
{
  m_data = FASTUIDRAWnew Params();
}

#define set_get_implement(name, type)                   \
  fastuidraw::RadialGradientParams&                     \
  fastuidraw::RadialGradientParams::                    \
  name(type v)                                          \
  {                                                     \
    Params *d;                                          \
    assert(dynamic_cast<Params*>(m_data) != NULL);      \
    d = static_cast<Params*>(m_data);                   \
    d->m_##name = v;                                    \
    return *this;                                       \
  }                                                     \
                                                        \
  type                                                  \
  fastuidraw::RadialGradientParams::                    \
  name(void) const                                      \
  {                                                     \
    Params *d;                                          \
    assert(dynamic_cast<Params*>(m_data) != NULL);      \
    d = static_cast<Params*>(m_data);                   \
    return d->m_##name;                                 \
  }


set_get_implement(start_pt, const fastuidraw::vec2&)
set_get_implement(end_pt, const fastuidraw::vec2&)
set_get_implement(start_r, float)
set_get_implement(end_r, float)
set_get_implement(flags, uint32_t)
set_get_implement(color_stop_sequence, const fastuidraw::reference_counted_ptr<const fastuidraw::ColorStopSequenceOnAtlas>&)

#undef set_get_implement
