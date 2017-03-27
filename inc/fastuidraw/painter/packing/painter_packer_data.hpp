/*!
 * \file painter_packer_data.hpp
 * \brief file painter_packer_data.hpp
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

#pragma once

#include <fastuidraw/painter/painter_data.hpp>

namespace fastuidraw
{
/*!\addtogroup PainterPacking
  @{
 */

  /*!
    A PainterPackerData is the data parameters for drawing
    commans of PainterPacker.
   */
  class FASTUIDRAW_API PainterPackerData:public PainterData
  {
  public:
    /*!
      Ctor. Intitializes all fields as default nothings.
     */
    PainterPackerData(void)
    {}

    /*!
      Initializes those fiels coming from PainterData from
      a PainterData value.
      \param obj PainterData object from which to take value
     */
    explicit
    PainterPackerData(const PainterData &obj):
      PainterData(obj)
    {}

    /*!
      value for the clip equations.
     */
    value<PainterClipEquations> m_clip;

    /*!
      value for the transformation matrix.
     */
    value<PainterItemMatrix> m_matrix;
  };

/*! @} */

}
