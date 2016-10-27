/*!
 * \file transformation_matrix_params.hpp
 * \brief file transformation_matrix_params.hpp
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

#include <fastuidraw/util/vecN.hpp>
#include <fastuidraw/util/matrix.hpp>
#include <fastuidraw/painter/painter_shader_data.hpp>

namespace fastuidraw
{
/*!\addtogroup Painter
  @{
 */

  /*!
    Class to specify a repeat window for a brush, data is
    packed as according to TransformationMatrixParams::data_offset_t.
   */
  class TransformationMatrixParams:public PainterBrushShaderData
  {
  public:
    /*!
      Enumeration that provides offset for packing of
      data of a transformation matrix.
     */
    enum data_offset_t
      {
        m00_offset, /*!< offset for float2x2(0, 0) (packed at float) */
        m01_offset, /*!< offset for float2x2(0, 1) (packed at float) */
        m10_offset, /*!< offset for float2x2(1, 0) (packed at float) */
        m11_offset, /*!< offset for float2x2(1, 1) (packed at float) */
        data_size /*!< size of data */
      };

    /*!
      Ctor.
     */
    TransformationMatrixParams(void);

    /*!
      Gives the matrix to apply to the brush.
     */
    const float2x2&
    matrix(void) const;

    /*!
      Set the value returned by pos(void) const.
     */
    TransformationMatrixParams&
    matrix(const float2x2 &v);
  };

/*! @} */

} //namespace fastuidraw
