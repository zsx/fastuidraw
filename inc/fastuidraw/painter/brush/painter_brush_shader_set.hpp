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

#include <fastuidraw/painter/brush/painter_brush_shader.hpp>

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
      Enumeration to name a PainterBrushShader
      within a PainterBrushShaderSet
     */
    enum effect_t
      {
        /*!
          Modulate by a constant color value,
          specified by \ref PenParams.
         */
        color,

        /*!
          Modulate by a linear gradient,
          specified by \ref LinearGradientParams.
         */
        linear_gradient,

        /*!
          Modulate by a radial gradient,
          specified by \ref RadialGradientParams.
         */
        radial_gradient,

        /*!
          Modulate by an image,
          specified by \ref ImageParams.
         */
        image,

        /*!
          Apply a repeat window to the brush position,
          specified by \ref RepeateWindowParams.
         */
        repeat_window,

        /*!
          Apply a translation to the brush position,
          specified by \ref TransformationTranslationParams.
         */
        transformation_translation,

        /*!
          Apply a matrix to the brush position,
          specified by \ref TransformationMatrixParams.
         */
        transformation_matrix,

        /*!
          Apply many effect via a singe stage (usually
          gives better performance), specified by a
          UnifiedBrushParams.
         */
        unified_brush,
      };

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
      Returns the PainterBrushShader for a brush effect type.
     */
    const reference_counted_ptr<PainterBrushShader>&
    shader(enum effect_t tp) const;

    /*!
      Set the PainterBrushShader for a brush effect type.
     */
    PainterBrushShaderSet&
    shader(enum effect_t tp, const reference_counted_ptr<PainterBrushShader> &sh);

    /*!
      Returns the one plus the largest value for which
      shader(enum effect_t, const reference_counted_ptr<PainterBrushShader>&)
      was called.
     */
    unsigned int
    shader_count(void) const;

  private:
    void *m_d;
  };
/*! @} */
}
