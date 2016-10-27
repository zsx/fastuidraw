/*!
 * \file image_params_params.hpp
 * \brief file image_params_params.hpp
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
#include <fastuidraw/image.hpp>
#include <fastuidraw/painter/painter_shader_data.hpp>

namespace fastuidraw
{
/*!\addtogroup Painter
  @{
 */

  /*!
    Class to specify image parameters, data is packed
    as according to ImageParams::data_offset_t.
   */
  class ImageParams:public PainterBrushShaderData
  {
  public:
    /*!
      Enumeration specifying what filter to apply to an image
     */
    enum filter_t
      {
        /*!
          Indicates to use nearest filtering (i.e
          choose closest pixel). No requirement on
          Image::slack() when using this filtering
          option.
         */
        filter_nearest = 1,

        /*!
          Indicates to use bilinear filtering. Requires
          that Image::slack() is atleast 1, otherwise
          rendering results will be wrong.
         */
        filter_linear = 2,

        /*!
          Indicates to use bicubic filtering. Requires
          that Image::slack() is atleast 2, otherwise
          rendering results will be wrong.
         */
        filter_cubic = 3
      };

    /*!
      Bit packing for the master index tile of a Image
     */
    enum atlas_location_encoding
      {
        atlas_location_x_num_bits = 8,  /*!< number bits to encode Image::master_index_tile().x() */
        atlas_location_y_num_bits = 8,  /*!< number bits to encode Image::master_index_tile().y() */
        atlas_location_z_num_bits = 16, /*!< number bits to encode Image::master_index_tile().z() */

        atlas_location_x_bit0 = 0, /*!< bit where Image::master_index_tile().x() is encoded */

        /*!
          bit where Image::master_index_tile().y() is encoded
         */
        atlas_location_y_bit0 = atlas_location_x_num_bits,

        /*!
          bit where Image::master_index_tile().z() is encoded
        */
        atlas_location_z_bit0 = atlas_location_y_bit0 + atlas_location_y_num_bits,
      };

    /*!
      Bit packing for size of the image, Image::dimensions()
     */
    enum texel_xy_encoding
      {
        size_x_num_bits = 16, /*!< number bits to encode Image::dimensions().x() */
        size_y_num_bits = 16, /*!< number bits to encode Image::dimensions().y() */

        size_x_bit0 = 0, /*!< bit where Image::dimensions().x() is encoded */
        size_y_bit0 = size_x_num_bits, /*!< bit where Image::dimensions().y() is encoded */
      };

    /*!
      Encoding for misc bits on how to render an Image.
     */
    enum misc_bits_encoding
      {
        /*!
          Number bits used to store the value of
          Image::number_index_lookups()
         */
        number_index_lookups_num_bits = 16,

        /*!
          Number bits used to store the value of
          Image::slack().
         */
        slack_num_bits = 8,

        /*!
          Number bits to encode the filter applied
          to the image.
         */
        filter_num_bits = 8,

        /*!
          first bit used to store Image::number_index_lookups()
         */
        number_index_lookups_bit0 = 0,

        /*!
          first bit used to store Image::slack()
         */
        slack_bit0 = number_index_lookups_bit0 + number_index_lookups_num_bits,

        /*!
          first bit used to store the filter applied to the image
         */
        filter_bit0 = slack_bit0 + slack_num_bits,
      };

    /*!
      Offsets for image data packing. The ratio of the size of the
      image to the size of the master index is given by
      pow(I, Image::number_index_lookups). where I is given by
      ImageAtlas::index_tile_size().
     */
    enum data_offset_t
      {
        /*!
          Location of image (Image::master_index_tile()) in
          the image atlas is encoded in a single uint32. The bits
          are packed as according to \ref atlas_location_encoding
         */
        atlas_location_xyz_offset,

        /*!
          Width and height of the image (Image::dimensions())
          encoded in a single uint32. The bits are packed as according
          to \ref texel_xy_encoding
         */
        size_xy_offset,

        /*!
          Top left corner of start of image to use (for example
          using the entire image would be (0,0)). Both x and y
          start values are encoded into a simge uint32. Encoding
          is the same as size_xy_offset, see \ref texel_xy_encoding.
         */
        start_xy_offset,

        /*!
          Stores Image::slack(), Image::number_index_lookups()
          and what filter to apply encoded by \ref misc_bits_encoding
         */
        misc_offset,

        /*!
          Size of the data
         */
        data_size
      };

    /*!
      Ctor.
     */
    ImageParams(void);

     /*!
      Sets to source from a given image.
      \param im handle to image to use. If handle is invalid,
                then sets brush to not have an image.
      \param f filter to apply to image, only has effect if im
               is non-NULL
     */
    void
    image(const reference_counted_ptr<const Image> &im, enum filter_t f = filter_nearest);

    /*!
      Sets to source from a sub-rectangle of an image
      \param im handle to image to use
      \param xy top-left corner of sub-rectangle of image to use
      \param wh width and height of sub-rectangle of image to use
      \param f filter to apply to image, only has effect if im
               is non-NULL
     */
    void
    sub_image(const reference_counted_ptr<const Image> &im, uvec2 xy, uvec2 wh,
              enum filter_t f = filter_nearest);

    /*!
      Returns true if and only if passed image can
      be rendered correctly with the specified filter.
      \param im handle to image
      \param f image filter to which to with which test if
               im can be rendered
     */
    static
    bool
    filter_suitable_for_image(const reference_counted_ptr<const Image> &im,
                              enum filter_t f);

    /*!
      Returns the highest quality filter with which
      an image may be rendered.
      \param im image to which to query
     */
    static
    enum filter_t
    best_filter_for_image(const reference_counted_ptr<const Image> &im);

    /*!
      Returns the slack requirement for an image to
      be rendered correctly under a filter.
      \param f filter to query
     */
    static
    int
    slack_requirement(enum filter_t f);
  };

/*! @} */

} //namespace fastuidraw
