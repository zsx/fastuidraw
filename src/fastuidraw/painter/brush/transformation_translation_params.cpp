/*!
 * \file transformation_translation_params.cpp
 * \brief file transformation_translation_params.cpp
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
#include <fastuidraw/painter/brush/transformation_translation_params.hpp>

namespace
{
  class TTParams:public fastuidraw::PainterBrushShaderData::DataBase
  {
  public:
    TTParams(void):
      m_pos(0.0f, 0.0f)
    {}

    virtual
    fastuidraw::PainterBrushShaderData::DataBase*
    copy(void) const
    {
      return FASTUIDRAWnew TTParams(*this);
    }

    virtual
    unsigned int
    data_size(unsigned int alignment) const
    {
      return fastuidraw::round_up_to_multiple(fastuidraw::TransformationTranslationParams::data_size, alignment);
    }

    virtual
    void
    pack_data(unsigned int alignment, fastuidraw::c_array<fastuidraw::generic_data> dst) const
    {
      FASTUIDRAWunused(alignment);
      dst[fastuidraw::TransformationTranslationParams::x_offset].f = m_pos.x();
      dst[fastuidraw::TransformationTranslationParams::y_offset].f = m_pos.y();
    }

    fastuidraw::vec2 m_pos;
  };
}


//////////////////////////////////////
// TransformationTranslationParams methods
fastuidraw::TransformationTranslationParams::
TransformationTranslationParams(void)
{
  m_data = FASTUIDRAWnew TTParams();
}

#define set_get_implement(name, type)                                  \
  fastuidraw::TransformationTranslationParams&                         \
  fastuidraw::TransformationTranslationParams::                        \
  name(type v)                                                         \
  {                                                                    \
    TTParams *d;                                                       \
    assert(dynamic_cast<TTParams*>(m_data) != NULL);                   \
    d = static_cast<TTParams*>(m_data);                                \
    d->m_##name = v;                                                   \
    return *this;                                                      \
  }                                                                    \
                                                                       \
  type                                                                 \
  fastuidraw::TransformationTranslationParams::                        \
  name(void) const                                                     \
  {                                                                    \
    TTParams *d;                                                       \
    assert(dynamic_cast<TTParams*>(m_data) != NULL);                   \
    d = static_cast<TTParams*>(m_data);                                \
    return d->m_##name;                                                \
  }

set_get_implement(pos, const fastuidraw::vec2&)

#undef set_get_implement
