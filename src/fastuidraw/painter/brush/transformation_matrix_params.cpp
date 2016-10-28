/*!
 * \file transformation_matrix_params.cpp
 * \brief file transformation_matrix_params.cpp
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
#include <fastuidraw/painter/brush/transformation_matrix_params.hpp>

namespace
{
  class TMParams:public fastuidraw::PainterShaderData::DataBase
  {
  public:
    TMParams(void)
    {}

    virtual
    fastuidraw::PainterShaderData::DataBase*
    copy(void) const
    {
      return FASTUIDRAWnew TMParams(*this);
    }

    virtual
    unsigned int
    data_size(unsigned int alignment) const
    {
      return fastuidraw::round_up_to_multiple(fastuidraw::TransformationMatrixParams::data_size, alignment);
    }

    virtual
    void
    pack_data(unsigned int alignment, fastuidraw::c_array<fastuidraw::generic_data> dst) const
    {
      FASTUIDRAWunused(alignment);
      dst[fastuidraw::TransformationMatrixParams::m00_offset].f = m_matrix(0, 0);
      dst[fastuidraw::TransformationMatrixParams::m01_offset].f = m_matrix(0, 1);
      dst[fastuidraw::TransformationMatrixParams::m10_offset].f = m_matrix(1, 0);
      dst[fastuidraw::TransformationMatrixParams::m11_offset].f = m_matrix(1, 1);
    }

    fastuidraw::float2x2 m_matrix;
  };
}


//////////////////////////////////////
// TransformationMatrixParams methods
fastuidraw::TransformationMatrixParams::
TransformationMatrixParams(void)
{
  m_data = FASTUIDRAWnew TMParams();
}

#define set_get_implement(name, type)                                  \
  fastuidraw::TransformationMatrixParams&                              \
  fastuidraw::TransformationMatrixParams::                             \
  name(type v)                                                         \
  {                                                                    \
    TMParams *d;                                                       \
    assert(dynamic_cast<TMParams*>(m_data) != NULL);                   \
    d = static_cast<TMParams*>(m_data);                                \
    d->m_##name = v;                                                   \
    return *this;                                                      \
  }                                                                    \
                                                                       \
  type                                                                 \
  fastuidraw::TransformationMatrixParams::                             \
  name(void) const                                                     \
  {                                                                    \
    TMParams *d;                                                       \
    assert(dynamic_cast<TMParams*>(m_data) != NULL);                   \
    d = static_cast<TMParams*>(m_data);                                \
    return d->m_##name;                                                \
  }

set_get_implement(matrix, const fastuidraw::float2x2&)

#undef set_get_implement
