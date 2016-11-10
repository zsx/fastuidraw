/*!
 * \file unified_brush_params.hpp
 * \brief file unified_brush_params.hpp
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
#include <fastuidraw/image.hpp>
#include <fastuidraw/colorstop_atlas.hpp>
#include <fastuidraw/painter/brush/painter_brush_shader_data.hpp>
#include <fastuidraw/painter/brush/image_params.hpp>

namespace fastuidraw
{
/*!\addtogroup Painter
  @{
 */

  /*!
    Class to specify many common stages used in a brush
    with a single brush stage. Each effect is optional,
    the effects (when present) are applied in the order:
    # Constant Color
    # Transformation matrix
    # Transformation translation
    # Repeat window
    # Linear OR Radial Gradient
    # Image

    In addition, the data is packed in that exact order too,
    preceeded by the shader brush value.
   */
  class UnifiedBrushParams:public PainterBrushShaderData
  {
  public:
    /*!
      Enumeration describing the roles of the bits for
      PainterShaderData::DataBase::uint_value().
     */
    enum active_bits_t
      {
        /*!
          Bit up if pen color is present
         */
        pen_bit,

        /*!
          Bit is up if image is present
         */
        image_bit,

        /*!
          Bit is up if a linear gradient is present
         */
        linear_gradient_bit,

        /*!
          bit is up if a radial gradient is present
         */
        radial_gradient_bit,

        /*!
          Bit up if the brush has a repeat window
         */
        repeat_window_bit,

        /*!
          Bit up if transformation 2x2 matrix is present
         */
        transformation_translation_bit,

        /*!
          Bit up is translation is present
         */
        transformation_matrix_bit,

        /*!
          Number of bits needed to encode what is active
         */
        number_bits,
      };

    /*!
      Masks generated from shader_bits, use these masks on the
      return value of PainterShaderData::DataBase::uint_value()
      to get what features are active on the UnifiedBrushParams.
     */
    enum active_masks_t
      {
        /*!
          bit mask for if pen color is used in brush.
         */
        pen_mask = FASTUIDRAW_MASK(pen_bit, 1),

        /*!
          bit mask for if image is used in brush.
         */
        image_mask = FASTUIDRAW_MASK(image_bit, 1),

        /*!
          bit for if linear_gradient is used in brush
         */
        linear_gradient_mask = FASTUIDRAW_MASK(linear_gradient_bit, 1),

        /*!
          bit for if radial_gradient is used in brush
         */
        radial_gradient_mask = FASTUIDRAW_MASK(radial_gradient_bit, 1),

        /*!
          bit for if repeat_window is used in brush
         */
        repeat_window_mask = FASTUIDRAW_MASK(repeat_window_bit, 1),

        /*!
          bit mask for if translation is used in brush
         */
        transformation_translation_mask = FASTUIDRAW_MASK(transformation_translation_bit, 1),

        /*!
          bit mask for if matrix is used in brush
         */
        transformation_matrix_mask = FASTUIDRAW_MASK(transformation_matrix_bit, 1),
      };

    /*!
      Enumeration giving the packing order for data of a brush.
      Each enumeration is an entry and when data is packed each
      entry starts on a multiple of the alignment (see
      PainterBackend::ConfigurationBase::alignment()) to the
      destination packing store.
     */
    enum packing_order_t
      {
        /*!
          Pen packed first via \ref PenParams.
         */
        pen_packing,

        /*!
          image packing via \ref ImageParams.
         */
        image_packing,

        /*!
          gradient packing via LinearGradientParams or
          RadialGradientParams.
         */
        gradient_packing,

        /*!
          repeat window packing via \ref RepeatWindowParams
         */
        repeat_window_packing,

        /*!
          transformation translation via \ref
          TransformationTranslationParams.
         */
        transformation_translation_packing,

        /*!
          transformation matrix via \ref
          TransformationMatrixParams.
         */
        transformation_matrix_packing,
      };

    /*!
      Ctor. Initializes  to have no image, no gradient, no pen,
      no repeat window and no transformation.
     */
    UnifiedBrushParams(void);

    /*!
      Reset the brush to initial conditions.
     */
    UnifiedBrushParams&
    reset(void);

    /*!
      Returns the pen color to apply to the brush,
      if no pen is active returns
      vec4(1.0, 1.0, 1.0, 1.0);
     */
    const vec4&
    pen(void) const;

    /*!
      Set to use a pen and the color of the pen.
     */
    UnifiedBrushParams&
    pen(const vec4 &v);

    /*!
      Set to use a pen and the color of the pen.
     */
    UnifiedBrushParams&
    pen(float r, float g, float b, float a = 1.0f)
    {
      return pen(vec4(r, g, b, a));
    }

    /*!
      Set to not use a pen.
     */
    UnifiedBrushParams&
    no_pen(void);

    /*!
      Sets the brush to have an image.
      \param im handle to image to use. If handle is invalid,
                then sets brush to not have an image.
      \param f filter to apply to image, only has effect if im
               is non-NULL
     */
    UnifiedBrushParams&
    image(const reference_counted_ptr<const Image> &im,
          enum ImageParams::filter_t f = ImageParams::filter_nearest);

    /*!
      Set the brush to source from a sub-rectangle of an image
      \param im handle to image to use
      \param xy top-left corner of sub-rectangle of image to use
      \param wh width and height of sub-rectangle of image to use
      \param f filter to apply to image, only has effect if im
               is non-NULL
     */
    UnifiedBrushParams&
    sub_image(const reference_counted_ptr<const Image> &im, uvec2 xy, uvec2 wh,
              enum ImageParams::filter_t f = ImageParams::filter_nearest);

    /*!
      Sets the brush to not have an image.
     */
    UnifiedBrushParams&
    no_image(void);

    /*!
      Sets the brush to have a linear gradient.
      \param cs color stops for gradient. If handle is invalid,
                then sets brush to not have a gradient.
      \param start_p start position of gradient
      \param end_p end position of gradient.
      \param repeat if true, repeats the gradient, if false then
                    clamps the gradient
     */
    UnifiedBrushParams&
    linear_gradient(const reference_counted_ptr<const ColorStopSequenceOnAtlas> &cs,
                    const vec2 &start_p, const vec2 &end_p, bool repeat);
    /*!
      Sets the brush to have a radial gradient.
      \param cs color stops for gradient. If handle is invalid,
                then sets brush to not have a gradient.
      \param start_p start position of gradient
      \param start_r starting radius of radial gradient
      \param end_p end position of gradient.
      \param end_r ending radius of radial gradient
      \param repeat if true, repeats the gradient, if false then
                    clamps the gradient
     */
    UnifiedBrushParams&
    radial_gradient(const reference_counted_ptr<const ColorStopSequenceOnAtlas> &cs,
                    const vec2 &start_p, float start_r,
                    const vec2 &end_p, float end_r, bool repeat);

    /*!
      Sets the brush to not have a gradient.
     */
    UnifiedBrushParams&
    no_gradient(void);

    /*!
      Sets the brush to have a translation in its transformation.
      \param p translation value for brush transformation
     */
    UnifiedBrushParams&
    transformation_translate(const vec2 &p);

    /*!
      Sets the brush to have a matrix in its transformation.
      \param m matrix value for brush transformation
     */
    UnifiedBrushParams&
    transformation_matrix(const float2x2 &m);

    /*!
      Sets the brush to have a matrix and translation in its
      transformation
      \param p translation value for brush transformation
      \param m matrix value for brush transformation
     */
    UnifiedBrushParams&
    transformation(const vec2 &p, const float2x2 &m)
    {
      transformation_translate(p);
      transformation_matrix(m);
      return *this;
    }

    /*!
      Sets the brush to have no translation in its transformation.
     */
    UnifiedBrushParams&
    no_transformation_translation(void);

    /*!
      Sets the brush to have no matrix in its transformation.
     */
    UnifiedBrushParams&
    no_transformation_matrix(void);

    /*!
      Sets the brush to have no transformation.
     */
    UnifiedBrushParams&
    no_transformation(void)
    {
      no_transformation_translation();
      no_transformation_matrix();
      return *this;
    }

    /*!
      Sets the brush to have a repeat window
      \param pos location of repeat window
      \param size of repeat window
     */
    UnifiedBrushParams&
    repeat_window(const vec2 &pos, const vec2 &size);

    /*!
      Sets the brush to not have a repeat window
     */
    UnifiedBrushParams&
    no_repeat_window(void);
  };

/*! @} */

} //namespace fastuidraw
