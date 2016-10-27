/*!
 * \file image_params.cpp
 * \brief file image_params.cpp
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
#include <fastuidraw/painter/brush/image_params.hpp>

namespace
{
  class Params:public fastuidraw::PainterShaderData::DataBase
  {
  public:
    Params(void):
      m_image_start(0, 0),
      m_image_size(1, 1),
      m_filter(fastuidraw::ImageParams::filter_nearest)
    {}

    virtual
    fastuidraw::PainterShaderData::DataBase*
    copy(void) const
    {
      return FASTUIDRAWnew Params(*this);
    }

    virtual
    unsigned int
    data_size(unsigned int alignment) const
    {
      using namespace fastuidraw;
      return round_up_to_multiple(ImageParams::data_size, alignment);
    }

    virtual
    void
    pack_data(unsigned int alignment, fastuidraw::c_array<fastuidraw::generic_data> dst) const
    {
      using namespace fastuidraw;

      FASTUIDRAWunused(alignment);
      if(m_image)
        {
          uvec3 loc(m_image->master_index_tile());
          dst[ImageParams::atlas_location_xyz_offset].u =
            pack_bits(ImageParams::atlas_location_x_bit0, ImageParams::atlas_location_x_num_bits, loc.x())
            | pack_bits(ImageParams::atlas_location_y_bit0, ImageParams::atlas_location_y_num_bits, loc.y())
            | pack_bits(ImageParams::atlas_location_z_bit0, ImageParams::atlas_location_z_num_bits, loc.z());

          dst[ImageParams::size_xy_offset].u =
            pack_bits(ImageParams::size_x_bit0, ImageParams::size_x_num_bits, m_image_size.x())
            | pack_bits(ImageParams::size_y_bit0, ImageParams::size_y_num_bits, m_image_size.y());

          dst[ImageParams::start_xy_offset].u =
            pack_bits(ImageParams::size_x_bit0, ImageParams::size_x_num_bits, m_image_start.x())
            | pack_bits(ImageParams::size_y_bit0, ImageParams::size_y_num_bits, m_image_start.y());

          uint32_t lookups(m_image->number_index_lookups());
          uint32_t slack(m_image->slack());

          dst[ImageParams::misc_offset].u =
            pack_bits(ImageParams::number_index_lookups_bit0, ImageParams::number_index_lookups_num_bits, lookups)
            | pack_bits(ImageParams::slack_bit0, ImageParams::slack_num_bits, slack)
            | pack_bits(ImageParams::filter_bit0, ImageParams::filter_num_bits, m_filter);
        }
      else
        {
          dst[ImageParams::atlas_location_xyz_offset].u = 0u;
          dst[ImageParams::size_xy_offset].u = 0u;
          dst[ImageParams::start_xy_offset].u = 0u;
          dst[ImageParams::misc_offset].u = 0u;
        }
    }

    fastuidraw::reference_counted_ptr<const fastuidraw::Image> m_image;
    fastuidraw::uvec2 m_image_start, m_image_size;
    enum fastuidraw::ImageParams::filter_t m_filter;
  };
}

////////////////////////////////////////
// fastuidraw::ImageParams methods
fastuidraw::ImageParams::
ImageParams(void)
{
  m_data = FASTUIDRAWnew Params();
}

void
fastuidraw::ImageParams::
image(const reference_counted_ptr<const Image> &im, enum filter_t f)
{
  if(im)
    {
      sub_image(im, uvec2(0,0), uvec2(im->dimensions()), f);
    }
  else
    {
      sub_image(im, uvec2(0,0), uvec2(1u, 1u), f);
    }
}

void
fastuidraw::ImageParams::
sub_image(const reference_counted_ptr<const Image> &im,
          uvec2 xy, uvec2 wh, enum filter_t f)
{
  Params *d;
  assert(dynamic_cast<Params*>(m_data) != NULL);
  d = static_cast<Params*>(m_data);
  d->m_image = im;
  d->m_image_start = xy;
  d->m_image_size = wh;
  d->m_filter = f;
}

enum fastuidraw::ImageParams::filter_t
fastuidraw::ImageParams::
best_filter_for_image(const reference_counted_ptr<const Image> &im)
{
  return im ?
    static_cast<enum filter_t>(t_min(im->slack() + 1,
                                     static_cast<unsigned int>(filter_cubic))) :
    filter_nearest;
}

bool
fastuidraw::ImageParams::
filter_suitable_for_image(const reference_counted_ptr<const Image> &im,
                          enum filter_t f)
{
  assert(f >= filter_nearest);
  assert(f <= filter_cubic);
  return im && im->slack() >= static_cast<unsigned int>(f) - 1;
}

int
fastuidraw::ImageParams::
slack_requirement(enum filter_t f)
{
  assert(f >= filter_nearest);
  assert(f <= filter_cubic);
  return f - 1;
}
