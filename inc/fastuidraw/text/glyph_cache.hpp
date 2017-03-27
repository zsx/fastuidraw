/*!
 * \file glyph_cache.hpp
 * \brief file glyph_cache.hpp
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
#include <fastuidraw/text/glyph_atlas.hpp>
#include <fastuidraw/text/font.hpp>
#include <fastuidraw/text/glyph_layout_data.hpp>
#include <fastuidraw/text/glyph.hpp>

namespace fastuidraw
{
/*!\addtogroup Text
  @{
*/

  /*!
    A GlyphCache represents a cache of glyphs and manages the uploading
    of the data to a GlyphAtlas. Methods are NOT thread safe.
   */
  class FASTUIDRAW_API GlyphCache:public reference_counted<GlyphCache>::default_base
  {
  public:
    /*!
      Ctor
      \param patlas GlyphAtlas to store glyph data
     */
    explicit
    GlyphCache(reference_counted_ptr<GlyphAtlas> patlas);

    ~GlyphCache();

    /*!
      Fetch, and if necessay create and store, a glyph given a
      glyph code of a font and a GlyphRender specifying how
      to render the glyph.
     */
    Glyph
    fetch_glyph(GlyphRender render,
                const reference_counted_ptr<const FontBase> &font,
                uint32_t glyph_code);

    /*!
      Removes a glyph from the -CACHE-, i.e. the GlyphCache,
      thus to use that glyph again requires calling fetch_glyph()
      (and thus fetching a new value for Glyph).
     */
    void
    delete_glyph(Glyph);

    /*!
      Call to clear the backing GlyphAtlas. In doing so, the glyphs
      will no longer be uploaded to the GlyphAtlas and will need
      to be re-uploaded (see Glyph::upload_to_atlas()). The glyphs
      however are NOT removed from this GlyphCache. Thus, the return
      values of previous calls to create_glyph() are still valie, but
      they need to be re-uploaded to the GlyphAtlas with
      Glyph::upload_to_atlas().
     */
    void
    clear_atlas(void);

    /*!
      Clear this GlyphCache and the GlyphAtlas. Essentially NUKE.
     */
    void
    clear_cache(void);

  private:
    void *m_d;
  };
/*! @} */
} //namespace fastuidraw
