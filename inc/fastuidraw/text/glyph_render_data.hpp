/*!
 * \file glyph_render_data.hpp
 * \brief file glyph_render_data.hpp
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

#include <stdint.h>

#include <fastuidraw/util/util.hpp>
#include <fastuidraw/util/util.hpp>
#include <fastuidraw/util/vecN.hpp>
#include <fastuidraw/util/c_array.hpp>
#include <fastuidraw/text/glyph_atlas.hpp>

namespace fastuidraw
{
/*!\addtogroup Text
  @{
*/

  /*!
    Provides an enumeration of the rendering data for
    a glyph.
   */
  enum glyph_type
    {
      /*!
        Glyph is a coverage glyph, generated
        from a GlyphRenderDataCoverage. Glyph is not
        scalable.
       */
      coverage_glyph,

      /*!
        Glyph is a distance field glyph, generated
        from a GlyphRenderDataDistanceField. Glyph
        is scalable.
       */
      distance_field_glyph,

      /*!
        Glyph is a curvepair glyph, generated
        from a GlyphRenderDataCurvePair. Glyph
        is scalable.
       */
      curve_pair_glyph,

      /*!
        Tag to indicate invalid glyph type; the value is much
        larger than the last glyph type to allow for later ABI
        compatibility as more glyph types are added.
       */
      invalid_glyph = 0x1000,
    };

  /*!
    Specifies how to render a glyph
   */
  class FASTUIDRAW_API GlyphRender
  {
  public:
    /*!
      Ctor. Initializes m_type as coverage_glyph
      \param pixel_size value to which to initialize m_pixel_size
     */
    explicit
    GlyphRender(int pixel_size);

    /*!
      Ctor.
      \param t value to which to initialize m_type, value must be so
               that scalable() returns true
     */
    explicit
    GlyphRender(enum glyph_type t);

    GlyphRender(void);

    /*!
      How to render glyph.
     */
    enum glyph_type m_type;

    /*!
      Pixel size observed only if scalable()
      when passed m_type returns false.
     */
    int m_pixel_size;

    /*!
      Returns true if and only if the data for a glyph type
      is scalable, for example distance_field_glyph and
      curve_pair_glyph are scalable
     */
    static
    bool
    scalable(enum glyph_type tp);

    /*!
      Comparison operator.
      \param rhs value to which to compare against
     */
    bool
    operator<(const GlyphRender &rhs) const;

    /*!
      Comparison operator.
      \param rhs value to which to compare against
     */
    bool
    operator==(const GlyphRender &rhs) const;

    /*!
      Returns true if and only if this GlyphRender is valid
      to specify how to render a glyph.
     */
    bool
    valid(void) const;
  };

  /*!
    GlyphRenderData provides an interface to specify
    data used for rendering glyphs and to pack that data
    onto a GlyphAtlas.
   */
  class FASTUIDRAW_API GlyphRenderData:noncopyable
  {
  public:
    virtual
    ~GlyphRenderData();

    /*!
      To be implemented by a derived class to upload
      data to a GlyphAtlas
      \param atlas GlyphAtlas to which to upload
      \param atlas_location (output) location of primary texels (see Glyph::location())
      \param secondary_atlas_location (output) location of secondary texels (see
                                      Glyph::secondary_atlas_location()), a value for
                                      which GlyphLocation::valid() returns false
                                      indicates that there are no secondary texels.
      \param geometry_offset (output) location of geometry data, -1 indicates no such data
      \param geometry_length (output) number of elements, in units of GlyphAtlasGeometryBackingStoreBase::alignment(),
                                      of geometry data.
     */
    virtual
    enum fastuidraw::return_code
    upload_to_atlas(const reference_counted_ptr<GlyphAtlas> &atlas,
                    GlyphLocation &atlas_location,
                    GlyphLocation &secondary_atlas_location,
                    int &geometry_offset,
                    int &geometry_length) const = 0;

  };
/*! @} */
}
