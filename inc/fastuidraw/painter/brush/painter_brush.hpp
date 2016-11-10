/*!
 * \file painter_brush.hpp
 * \brief file painter_brush.hpp
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

#include <fastuidraw/util/reference_counted.hpp>
#include <fastuidraw/util/vecN.hpp>
#include <fastuidraw/util/matrix.hpp>
#include <fastuidraw/util/c_array.hpp>

#include <fastuidraw/image.hpp>
#include <fastuidraw/colorstop_atlas.hpp>
#include <fastuidraw/painter/brush/painter_brush_shader_set.hpp>
#include <fastuidraw/painter/brush/pen_params.hpp>
#include <fastuidraw/painter/brush/image_params.hpp>
#include <fastuidraw/painter/brush/linear_gradient_params.hpp>
#include <fastuidraw/painter/brush/radial_gradient_params.hpp>
#include <fastuidraw/painter/brush/repeat_window_params.hpp>
#include <fastuidraw/painter/brush/transformation_translation_params.hpp>
#include <fastuidraw/painter/brush/transformation_matrix_params.hpp>
#include <fastuidraw/painter/brush/unified_brush_params.hpp>

namespace fastuidraw
{
/*!\addtogroup Painter
  @{
 */

  /*!
    A PainterBrush defines a brush for emitting a color per
    fragment. A PainterBrush is composed of stages that a
    user adds to the PainterBrush, see add_brush_stage().
    A stage can modify the final color emitted and/or change
    the brush position used by later stages.
  */
  class PainterBrush
  {
  public:
    /*!
      Indicates that PainterPackedValue<PainterBrush> does
      NOT have a copy of the value that generated it.
     */
    typedef false_type packed_value_has_value;

    /*!
      Indicates that PainterPackedValue<PainterBrush> does
      have a value derived from PainterBrush value/
     */
    typedef true_type packed_value_has_derived_value;

    /*!
      The derived value type that a PainterPackedValue<PainterBrush>
      will store when created from a PainterBrush
     */
    class derived_value_type
    {
    public:
      derived_value_type(void);
      derived_value_type(const derived_value_type &obj);
      ~derived_value_type();

      derived_value_type&
      operator=(const derived_value_type &rhs);

      uint32_t
      number_stages(void) const;

    private:
      friend class PainterBrush;
      void *m_d;
    };

    /*!
      Ctor. Initializes to have no brush stages, i.e. brush
      always emits (1.0, 1.0, 1.0, 1.0) which is solid white.
     */
    PainterBrush(void);

    /*!
      Copy ctor.
     */
    PainterBrush(const PainterBrush &obj);

    ~PainterBrush();

    /*!
      Assignment operator.
     */
    PainterBrush&
    operator=(const PainterBrush &rhs);

    /*!
      Reset the brush to initial conditions.
     */
    PainterBrush&
    reset(void);

    /*!
      Adds a stage to the brush specifying the shader
      and shader data.
      \param shader what PainterBrushShader to use for the stage
      \param data the data used by the PainterBrushShader, data is
                  COPIED to the PainterBrush.
      \param out_stage if non-NULL, the index to feed stage_data()
                       and stage_data_base() to get the instance of
                       the data of the added stage.
     */
    PainterBrush&
    add_brush_stage(const reference_counted_ptr<PainterBrushShader> &shader,
                    const PainterBrushShaderData &data,
                    unsigned int *out_stage = NULL);
    /*!
      Add constant color stage by which to modulate.
      \param pen PenParams that defines the constant color
      \param out_stage if non-NULL, the index to feed stage_data()
                       and stage_data_base() to get the instance of
                       the data of the added stage.
     */
    PainterBrush&
    add_brush_stage(const PenParams &pen, unsigned int *out_stage = NULL);

    /*!
      Add a brush stage by which to modulate by an image using brush
      position to specify what texel to fetch from an image.
      \param image ImageParams that specifies what image to use and how
      \param out_stage if non-NULL, the index to feed stage_data()
                       and stage_data_base() to get the instance of
                       the data of the added stage.
     */
    PainterBrush&
    add_brush_stage(const ImageParams &image, unsigned int *out_stage = NULL);

    /*!
      Add a stage to modulate the color by a linear gradient using
      the brush position to compute the color stop position.
      \param gradient LinearGradientParams giving position of points
                      defining the linear gradient and what color
                      stop sequence to use
      \param out_stage if non-NULL, the index to feed stage_data()
                       and stage_data_base() to get the instance of
                       the data of the added stage.
     */
    PainterBrush&
    add_brush_stage(const LinearGradientParams &gradient, unsigned int *out_stage = NULL);

    /*!
      Add a stage to modulate the color by a radial gradient using
      the brush position to compute the color stop position.
      \param gradient RadialGradientParams giving centers and radii
                      defining the radial gradient and what color
                      stop sequence to use
      \param out_stage if non-NULL, the index to feed stage_data()
                       and stage_data_base() to get the instance of
                       the data of the added stage.
     */
    PainterBrush&
    add_brush_stage(const RadialGradientParams &gradient, unsigned int *out_stage = NULL);

    /*!
      Add a brush stage to modify the brush position to apply
      a repeat window to the brush position.
      \param repeat_window provides the location and dimensions of
                           the repeat window to apply
      \param out_stage if non-NULL, the index to feed stage_data()
                       and stage_data_base() to get the instance of
                       the data of the added stage.
     */
    PainterBrush&
    add_brush_stage(const RepeatWindowParams &repeat_window, unsigned int *out_stage = NULL);

    /*!
      Add a brush stage that modifies the brush position by appying
      a translation to it.
      \param translation provides the vector by which to translate the
                         brush position
      \param out_stage if non-NULL, the index to feed stage_data()
                       and stage_data_base() to get the instance of
                       the data of the added stage.
     */
    PainterBrush&
    add_brush_stage(const TransformationTranslationParams &translation,
                    unsigned int *out_stage = NULL);

    /*!
      Add a brush stage to apply a 2x2 matrix multiply to
      the brush position
      \param matrix provides the 2x2 matrix to apply to the brush
                    position.
      \param out_stage if non-NULL, the index to feed stage_data()
                       and stage_data_base() to get the instance of
                       the data of the added stage.
     */
    PainterBrush&
    add_brush_stage(const TransformationMatrixParams &matrix,
                    unsigned int *out_stage = NULL);

    /*!
      Add a brush stage to unified brush stage
      \param unified provides a unified brush
      \param out_stage if non-NULL, the index to feed stage_data()
                       and stage_data_base() to get the instance of
                       the data of the added stage.
     */
    PainterBrush&
    add_brush_stage(const UnifiedBrushParams &unified, unsigned int *out_stage = NULL);

    /*!
      Returns the number of stages for the PainterBrush.
     */
    uint32_t
    number_stages(void) const;

    /*!
      Returns a reference to the shader data for the named
      stage.
      \param stage which stage to get
     */
    PainterBrushShaderData&
    stage_data_base(unsigned int stage);

    /*!
      Returns a reference to the shader data for the named
      stage.
      \tparam T type to which to case the PainterBrushShaderData value to
      \param stage which stage to get
     */
    template<typename T>
    T&
    stage_data(unsigned int stage)
    {
      return static_cast<T&>(stage_data_base(stage));
    }

    /*!
      Returns a reference to the shader data for the named
      stage.
      \param stage which stage to get
     */
    const PainterBrushShaderData&
    stage_data_base(unsigned int stage) const;

    /*!
      Returns a reference to the shader data for the named
      stage casted to a named type.
      \tparam T type to which to case the PainterBrushShaderData value to
      \param stage which stage to get
     */
    template<typename T>
    const T&
    stage_data(unsigned int stage) const
    {
      return static_cast<const T&>(stage_data_base(stage));
    }

    /*!
      Returns the length of the data needed to encode the data.
      Data is padded to be multiple of alignment.
      \param alignment alignment of the data store
                       in units of generic_data, see
                       PainterBackend::ConfigurationBase::alignment()
     */
    unsigned int
    data_size(unsigned int alignment) const;

    /*!
      Encodes the data. Data is packed in the order
      specified by \ref packing_order_t.
      Data is padded to be multiple of alignment and also
      sub-data of brush is padded to be along alignment
      boundaries.
      \param dst location to which to encode the brush
      \param alignment alignment of the data store, see
                       PainterBackend::ConfigurationBase::alignment(void) const
     */
    void
    pack_data(const PainterBrushShaderSet &shader_set,
              unsigned int alignment, c_array<generic_data> dst) const;

    /*!
      Returns the derived values stored by PainterPackedValue<PainterBrush>
     */
    const derived_value_type&
    derived_value(void) const;

  private:
    void *m_d;
  };
/*! @} */
}
