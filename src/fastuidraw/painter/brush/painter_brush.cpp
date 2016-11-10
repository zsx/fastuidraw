/*!
 * \file painter_brush.cpp
 * \brief file painter_brush.cpp
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

#include <vector>
#include <algorithm>
#include <fastuidraw/painter/brush/painter_brush.hpp>
#include "../../private/util_private.hpp"

namespace
{
  class BrushStage
  {
  public:
    void
    set(enum fastuidraw::PainterBrushShaderSet::effect_t ef,
        const fastuidraw::PainterBrushShaderData &d,
        bool *b)
    {
      m_effect = ef;
      m_data = d;
      m_data.dirty_marker(b);
    }

    void
    set(const fastuidraw::reference_counted_ptr<fastuidraw::PainterBrushShader> &sh,
        const fastuidraw::PainterBrushShaderData &d,
        bool *b)
    {
      m_shader = sh;
      m_data = d;
      m_data.dirty_marker(b);
    }

    enum fastuidraw::PainterBrushShaderSet::effect_t m_effect;
    fastuidraw::reference_counted_ptr<fastuidraw::PainterBrushShader> m_shader;
    fastuidraw::PainterBrushShaderData m_data;
  };

  class PainterBrushPrivate
  {
  public:
    std::vector<BrushStage> m_stages;
    fastuidraw::PainterBrush::derived_value_type m_derived_value;
    bool m_derived_value_dirty;
    unsigned int m_brush_ID;
  };

  class DerivedValuePrivate
  {
  public:
    typedef fastuidraw::Image Image;
    typedef fastuidraw::reference_counted_ptr<const Image> ImageRef;
    typedef fastuidraw::ColorStopSequenceOnAtlas ColorStops;
    typedef fastuidraw::reference_counted_ptr<const ColorStops> ColorStopsRef;

    DerivedValuePrivate(void):
      m_number_stages(0u)
    {}

    void
    set_values(const std::vector<BrushStage> &stages);

    uint32_t m_number_stages;
    std::vector<ImageRef> m_images;
    std::vector<ColorStopsRef> m_color_stops;
  };
}

//////////////////////////////////
// DerivedValuePrivate methods
void
DerivedValuePrivate::
set_values(const std::vector<BrushStage> &stages)
{
  m_number_stages = stages.size();
  m_images.clear();
  m_color_stops.clear();
  for(unsigned int i = 0, endi = stages.size(); i < endi; ++i)
    {
      fastuidraw::const_c_array<ImageRef> imgs;
      fastuidraw::const_c_array<ColorStopsRef> cs;
      const fastuidraw::PainterBrushShaderData &data(stages[i].m_data);

      imgs = data.images();
      if(!imgs.empty())
        {
          unsigned int prev_size(m_images.size());

          m_images.resize(prev_size + imgs.size());
          std::copy(imgs.begin(), imgs.end(), m_images.begin() + prev_size);
        }

      cs = data.color_stops();
      if(!cs.empty())
        {
          unsigned int prev_size(m_color_stops.size());

          m_color_stops.resize(prev_size + cs.size());
          std::copy(cs.begin(), cs.end(), m_color_stops.begin() + prev_size);
        }
    }
}

////////////////////////////////////
// derived_value_type methods
fastuidraw::PainterBrush::derived_value_type::
derived_value_type(void)
{
  m_d = FASTUIDRAWnew DerivedValuePrivate();
}

fastuidraw::PainterBrush::derived_value_type::
derived_value_type(const derived_value_type &obj)
{
  m_d = FASTUIDRAWnew DerivedValuePrivate(*reinterpret_cast<DerivedValuePrivate*>(obj.m_d));
}

fastuidraw::PainterBrush::derived_value_type::
~derived_value_type()
{
  DerivedValuePrivate *d;
  d = reinterpret_cast<DerivedValuePrivate*>(m_d);
  FASTUIDRAWdelete(d);
}

fastuidraw::PainterBrush::derived_value_type&
fastuidraw::PainterBrush::derived_value_type::
operator=(const derived_value_type &rhs)
{
  if(this != &rhs)
    {
      *reinterpret_cast<DerivedValuePrivate*>(m_d) = *reinterpret_cast<DerivedValuePrivate*>(rhs.m_d);
    }
  return *this;
}

uint32_t
fastuidraw::PainterBrush::derived_value_type::
number_stages(void) const
{
  DerivedValuePrivate *d;
  d = reinterpret_cast<DerivedValuePrivate*>(m_d);
  return d->m_number_stages;
}

////////////////////////////////////
// fastuidraw::PainterBrush methods
fastuidraw::PainterBrush::
PainterBrush(void)
{
  m_d = FASTUIDRAWnew PainterBrushPrivate();
}

fastuidraw::PainterBrush::
PainterBrush(const PainterBrush &obj)
{
  PainterBrushPrivate *obj_d;
  obj_d = reinterpret_cast<PainterBrushPrivate*>(obj.m_d);
  m_d = FASTUIDRAWnew PainterBrushPrivate(*obj_d);
}

fastuidraw::PainterBrush::
~PainterBrush()
{
  PainterBrushPrivate *d;
  d = reinterpret_cast<PainterBrushPrivate*>(m_d);

  FASTUIDRAWdelete(d);
  m_d = NULL;
}

unsigned int
fastuidraw::PainterBrush::
data_size(unsigned int alignment) const
{
  PainterBrushPrivate *d;
  unsigned int return_value;

  d = reinterpret_cast<PainterBrushPrivate*>(m_d);
  return_value = round_up_to_multiple(d->m_stages.size(), alignment);
  for(unsigned int i = 0, endi = d->m_stages.size(); i < endi; ++i)
    {
      return_value += d->m_stages[i].m_data.data_size(alignment);
    }
  return return_value;
}

void
fastuidraw::PainterBrush::
pack_data(const PainterBrushShaderSet &shader_set,
          unsigned int alignment, c_array<generic_data> dst) const
{
  unsigned int data_start, current(0u);
  c_array<generic_data> dst_data, dst_shaders;
  PainterBrushPrivate *d;

  d = reinterpret_cast<PainterBrushPrivate*>(m_d);
  data_start = round_up_to_multiple(d->m_stages.size(), alignment);
  dst_data = dst.sub_array(data_start);
  dst_shaders = dst.sub_array(0, data_start);

  generic_data zero;
  zero.u = 0u;
  std::fill(dst_shaders.begin() + d->m_stages.size(),
            dst_shaders.begin() + data_start,
            zero);

  for(unsigned int i = 0, endi = d->m_stages.size(); i < endi; ++i)
    {
      unsigned int sz;
      uint32_t sh_flags, sh;

      sz = d->m_stages[i].m_data.data_size(alignment);
      d->m_stages[i].m_data.pack_data(alignment, dst_data.sub_array(current, sz));
      current += sz;

      sh_flags = d->m_stages[i].m_data.shader_flags();
      if(d->m_stages[i].m_shader)
        {
          sh = d->m_stages[i].m_shader->ID();
        }
      else
        {
          sh = shader_set.shader(d->m_stages[i].m_effect)->ID();
        }
      dst_shaders[i].u = sh | (sh_flags << 16u);
    }
  assert(current == dst_data.size());
}

fastuidraw::PainterBrush&
fastuidraw::PainterBrush::
reset(void)
{
  PainterBrushPrivate *d;
  d = reinterpret_cast<PainterBrushPrivate*>(m_d);
  d->m_stages.clear();
  d->m_derived_value_dirty = true;
  return *this;
}

const fastuidraw::PainterBrush::derived_value_type&
fastuidraw::PainterBrush::
derived_value(void) const
{
  PainterBrushPrivate *d;
  d = reinterpret_cast<PainterBrushPrivate*>(m_d);
  if(d->m_derived_value_dirty)
    {
      DerivedValuePrivate *q;
      q = reinterpret_cast<DerivedValuePrivate*>(d->m_derived_value.m_d);
      q->set_values(d->m_stages);
      d->m_derived_value_dirty = false;
    }
  return d->m_derived_value;
}


fastuidraw::PainterBrush&
fastuidraw::PainterBrush::
add_brush_stage(const reference_counted_ptr<PainterBrushShader> &shader,
                const PainterBrushShaderData &data, unsigned int *out_stage)
{
  PainterBrushPrivate *d;
  d = reinterpret_cast<PainterBrushPrivate*>(m_d);

  unsigned int slot(d->m_stages.size());
  d->m_stages.push_back(BrushStage());
  d->m_stages.back().set(shader, data, &d->m_derived_value_dirty);
  d->m_derived_value_dirty = true;
  if(out_stage != NULL)
    {
      *out_stage = slot;
    }
  return *this;
}

fastuidraw::PainterBrush&
fastuidraw::PainterBrush::
add_brush_stage(const PenParams &data, unsigned int *out_stage)
{
  PainterBrushPrivate *d;
  d = reinterpret_cast<PainterBrushPrivate*>(m_d);

  unsigned int slot(d->m_stages.size());
  d->m_stages.push_back(BrushStage());
  d->m_stages.back().set(PainterBrushShaderSet::color, data, &d->m_derived_value_dirty);
  d->m_derived_value_dirty = true;
  if(out_stage != NULL)
    {
      *out_stage = slot;
    }
  return *this;
}

fastuidraw::PainterBrush&
fastuidraw::PainterBrush::
add_brush_stage(const ImageParams &data, unsigned int *out_stage)
{
  PainterBrushPrivate *d;
  d = reinterpret_cast<PainterBrushPrivate*>(m_d);

  unsigned int slot(d->m_stages.size());
  d->m_stages.push_back(BrushStage());
  d->m_stages.back().set(PainterBrushShaderSet::image, data,
                         &d->m_derived_value_dirty);
  d->m_derived_value_dirty = true;
  if(out_stage != NULL)
    {
      *out_stage = slot;
    }
  return *this;
}

fastuidraw::PainterBrush&
fastuidraw::PainterBrush::
add_brush_stage(const LinearGradientParams &data, unsigned int *out_stage)
{
  PainterBrushPrivate *d;
  d = reinterpret_cast<PainterBrushPrivate*>(m_d);

  unsigned int slot(d->m_stages.size());
  d->m_stages.push_back(BrushStage());
  d->m_stages.back().set(PainterBrushShaderSet::linear_gradient, data,
                         &d->m_derived_value_dirty);
  d->m_derived_value_dirty = true;
  if(out_stage != NULL)
    {
      *out_stage = slot;
    }
  return *this;
}

fastuidraw::PainterBrush&
fastuidraw::PainterBrush::
add_brush_stage(const RadialGradientParams &data, unsigned int *out_stage)
{
  PainterBrushPrivate *d;
  d = reinterpret_cast<PainterBrushPrivate*>(m_d);

  unsigned int slot(d->m_stages.size());
  d->m_stages.push_back(BrushStage());
  d->m_stages.back().set(PainterBrushShaderSet::radial_gradient, data,
                         &d->m_derived_value_dirty);
  d->m_derived_value_dirty = true;
  if(out_stage != NULL)
    {
      *out_stage = slot;
    }
  return *this;
}

fastuidraw::PainterBrush&
fastuidraw::PainterBrush::
add_brush_stage(const RepeatWindowParams &data, unsigned int *out_stage)
{
  PainterBrushPrivate *d;
  d = reinterpret_cast<PainterBrushPrivate*>(m_d);

  unsigned int slot(d->m_stages.size());
  d->m_stages.push_back(BrushStage());
  d->m_stages.back().set(PainterBrushShaderSet::repeat_window, data,
                         &d->m_derived_value_dirty);
  d->m_derived_value_dirty = true;
  if(out_stage != NULL)
    {
      *out_stage = slot;
    }
  return *this;
}

fastuidraw::PainterBrush&
fastuidraw::PainterBrush::
add_brush_stage(const TransformationTranslationParams &data, unsigned int *out_stage)
{
  PainterBrushPrivate *d;
  d = reinterpret_cast<PainterBrushPrivate*>(m_d);

  unsigned int slot(d->m_stages.size());
  d->m_stages.push_back(BrushStage());
  d->m_stages.back().set(PainterBrushShaderSet::transformation_translation,
                         data, &d->m_derived_value_dirty);
  d->m_derived_value_dirty = true;
  if(out_stage != NULL)
    {
      *out_stage = slot;
    }
  return *this;
}

fastuidraw::PainterBrush&
fastuidraw::PainterBrush::
add_brush_stage(const TransformationMatrixParams &data, unsigned int *out_stage)
{
  PainterBrushPrivate *d;
  d = reinterpret_cast<PainterBrushPrivate*>(m_d);

  unsigned int slot(d->m_stages.size());
  d->m_stages.push_back(BrushStage());
  d->m_stages.back().set(PainterBrushShaderSet::transformation_matrix,
                         data, &d->m_derived_value_dirty);
  d->m_derived_value_dirty = true;
  if(out_stage != NULL)
    {
      *out_stage = slot;
    }
  return *this;
}

fastuidraw::PainterBrush&
fastuidraw::PainterBrush::
add_brush_stage(const UnifiedBrushParams &data, unsigned int *out_stage)
{
  PainterBrushPrivate *d;
  d = reinterpret_cast<PainterBrushPrivate*>(m_d);

  unsigned int slot(d->m_stages.size());
  d->m_stages.push_back(BrushStage());
  d->m_stages.back().set(PainterBrushShaderSet::unified_brush,
                         data, &d->m_derived_value_dirty);
  d->m_derived_value_dirty = true;
  if(out_stage != NULL)
    {
      *out_stage = slot;
    }
  return *this;
}

fastuidraw::PainterBrushShaderData&
fastuidraw::PainterBrush::
stage_data_base(unsigned int stage)
{
  PainterBrushPrivate *d;
  d = reinterpret_cast<PainterBrushPrivate*>(m_d);
  assert(stage < d->m_stages.size());
  return d->m_stages[stage].m_data;
}

const fastuidraw::PainterBrushShaderData&
fastuidraw::PainterBrush::
stage_data_base(unsigned int stage) const
{
  PainterBrushPrivate *d;
  d = reinterpret_cast<PainterBrushPrivate*>(m_d);
  assert(stage < d->m_stages.size());
  return d->m_stages[stage].m_data;
}

uint32_t
fastuidraw::PainterBrush::
number_stages(void) const
{
  PainterBrushPrivate *d;
  d = reinterpret_cast<PainterBrushPrivate*>(m_d);
  return d->m_stages.size();
}
