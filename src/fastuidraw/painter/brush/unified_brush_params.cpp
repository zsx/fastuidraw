/*!
 * \file unified_brush_params.cpp
 * \brief file unified_brush_params.cpp
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
#include <fastuidraw/painter/brush/unified_brush_params.hpp>

namespace
{
  class UParams:public fastuidraw::PainterBrushShaderData::DataBase
  {
  public:
    UParams(void):
      m_active_mask(0u)
    {}

    virtual
    fastuidraw::PainterBrushShaderData::DataBase*
    copy(void) const
    {
      return FASTUIDRAWnew UParams(*this);
    }

    virtual
    fastuidraw::const_c_array<ImageRef>
    images(void) const;

    virtual
    fastuidraw::const_c_array<ColorStopSequenceOnAtlasRef>
    color_stops(void) const;

    virtual
    unsigned int
    data_size(unsigned int alignment) const;

    virtual
    void
    pack_data(unsigned int alignment, fastuidraw::c_array<fastuidraw::generic_data> dst) const;

    virtual
    uint32_t
    uint_value(void) const
    {
      return m_active_mask;
    }

    uint32_t m_active_mask;
    PenParams m_pen;
    ImageParams m_image;
    LinearGradientParams m_linear_gradient;
    RadialGradientParams m_radial_gradient;
    RepeatWindowParams m_repeat_window;
    TransformationTranslationParams m_translation;
    TransformationMatrixParams m_matrix;
  };
}
