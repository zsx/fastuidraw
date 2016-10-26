/*!
 * \file painter_brush_shader.hpp
 * \brief file painter_brush_shader.hpp
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

#include <fastuidraw/painter/painter_brush_shader.hpp>

namespace fastuidraw
{

/*!\addtogroup Painter
  @{
 */

  /*!
    A PainterBrushShaderSet holds brush shaders for common
    brush operations.
   */
  class PainterBrushShaderSet
  {
  public:
    /*!
      Ctor
     */
    PainterBrushShaderSet(void);

    /*!
      Copy ctor.
     */
    PainterBrushShaderSet(const PainterBrushShaderSet &obj);

    ~PainterBrushShaderSet();

    /*!
      Assignment operator.
     */
    PainterBrushShaderSet&
    operator=(const PainterBrushShaderSet &rhs);

    /*!
      The brush shader for performing linear gradient.
     */
    const reference_counted_ptr<PainterBrushShader>&
    linear_gradient(void) const;

    /*!
      Set the value returned by linear_gradient(void) const.
     */
    PainterBrushShaderSet&
    linear_gradient(const reference_counted_ptr<PainterBrushShader> &sh);

    /*!
      The brush shader for performing radial gradient.
     */
    const reference_counted_ptr<PainterBrushShader>&
    radial_gradient(void) const;

    /*!
      Set the value returned by radial_gradient(void) const.
     */
    PainterBrushShaderSet&
    radial_gradient(const reference_counted_ptr<PainterBrushShader> &sh);

    /*!
      The brush shader for performing composite with an image.
     */
    const reference_counted_ptr<PainterBrushShader>&
    image(void) const;

    /*!
      Set the value returned by image(void) const.
     */
    PainterBrushShaderSet&
    image(const reference_counted_ptr<PainterBrushShader> &sh);

    /*!
      The brush shader for performing translation on the brush position.
     */
    const reference_counted_ptr<PainterBrushShader>&
    transformation_translation(void) const;

    /*!
      Set the value returned by translation(void) const.
     */
    PainterBrushShaderSet&
    transformation_translation(const reference_counted_ptr<PainterBrushShader> &sh);

    /*!
      The brush shader for performing a 2x2 matrix transformation
      on the brush position.
     */
    const reference_counted_ptr<PainterBrushShader>&
    transformation_matrix(void) const;

    /*!
      Set the value returned by (void) const.
     */
    PainterBrushShaderSet&
    transformation_matrix(const reference_counted_ptr<PainterBrushShader> &sh);

  private:
    void *m_d;
  };
/*! @} */
}
