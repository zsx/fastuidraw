/*!
 * \file linear_gradient_params.cpp
 * \brief file linear_gradient_params.cpp
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
#include <fastuidraw/painter/brush/linear_gradient_params.hpp>

namespace
{
  class LGParams:public fastuidraw::PainterBrushShaderData::DataBase
  {
  public:
    LGParams(void):
      m_start_pt(0.0f, 0.0f),
      m_end_pt(1.0f, 1.0f),
      m_flags(0u),
      m_color_stop_sequence()
    {}

    virtual
    fastuidraw::PainterBrushShaderData::DataBase*
    copy(void) const
    {
      return FASTUIDRAWnew LGParams(*this);
    }

    virtual
    fastuidraw::const_c_array<ColorStopSequenceOnAtlasRef>
    color_stops(void) const
    {
      if(m_color_stop_sequence)
        {
          fastuidraw::const_c_array<ColorStopSequenceOnAtlasRef> R(&m_color_stop_sequence, 1);
          return R;
        }
      else
        {
          return fastuidraw::const_c_array<ColorStopSequenceOnAtlasRef>();
        }
    }

    virtual
    unsigned int
    data_size(unsigned int alignment) const
    {
      return fastuidraw::round_up_to_multiple(fastuidraw::LinearGradientParams::data_size, alignment);
    }

    virtual
    void
    pack_data(unsigned int alignment, fastuidraw::c_array<fastuidraw::generic_data> dst) const
    {
      using namespace fastuidraw;

      FASTUIDRAWunused(alignment);
      dst[LinearGradientParams::start_pt_x_offset].f = m_start_pt.x();
      dst[LinearGradientParams::start_pt_y_offset].f = m_start_pt.y();
      dst[LinearGradientParams::end_pt_x_offset].f = m_end_pt.x();
      dst[LinearGradientParams::end_pt_y_offset].f = m_end_pt.y();
      dst[LinearGradientParams::flags_offset].u = m_flags;
      if(m_color_stop_sequence)
        {
          int d;
          float r;

          d = m_color_stop_sequence->atlas()->backing_store()->dimensions().x();
          r = 1.0f / static_cast<float>(d);

          dst[LinearGradientParams::color_stop_sequence_x_offset].f = r * static_cast<float>(m_color_stop_sequence->texel_location().x());
          dst[LinearGradientParams::color_stop_sequence_y_offset].f = r * static_cast<float>(m_color_stop_sequence->texel_location().y());
          dst[LinearGradientParams::color_stop_sequence_width_offset].f = r * static_cast<float>(m_color_stop_sequence->width());
        }
      else
        {
          dst[LinearGradientParams::color_stop_sequence_x_offset].f = 0.0f;
          dst[LinearGradientParams::color_stop_sequence_y_offset].f = 0.0f;
          dst[LinearGradientParams::color_stop_sequence_width_offset].f = 0.0f;
        }
    }

    fastuidraw::vec2 m_start_pt, m_end_pt;
    uint32_t m_flags;
    fastuidraw::reference_counted_ptr<const fastuidraw::ColorStopSequenceOnAtlas> m_color_stop_sequence;
  };
}

//////////////////////////////////////
// LinearGradientParams methods
fastuidraw::LinearGradientParams::
LinearGradientParams(void)
{
  m_data = FASTUIDRAWnew LGParams();
}

#define set_get_implement(name, type)                     \
  fastuidraw::LinearGradientParams&                       \
  fastuidraw::LinearGradientParams::                      \
  name(type v)                                            \
  {                                                       \
    LGParams *d;                                          \
    assert(dynamic_cast<LGParams*>(m_data) != NULL);      \
    d = static_cast<LGParams*>(m_data);                   \
    d->m_##name = v;                                      \
    return *this;                                         \
  }                                                       \
                                                          \
  type                                                    \
  fastuidraw::LinearGradientParams::                      \
  name(void) const                                        \
  {                                                       \
    LGParams *d;                                          \
    assert(dynamic_cast<LGParams*>(m_data) != NULL);      \
    d = static_cast<LGParams*>(m_data);                   \
    return d->m_##name;                                   \
  }


set_get_implement(start_pt, const fastuidraw::vec2&)
set_get_implement(end_pt, const fastuidraw::vec2&)
set_get_implement(flags, uint32_t)
set_get_implement(color_stop_sequence, const fastuidraw::reference_counted_ptr<const fastuidraw::ColorStopSequenceOnAtlas>&)

#undef set_get_implement
