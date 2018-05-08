#pragma once

#include "fastuidraw/gl_backend/colorstop_atlas_gl.hpp"
#include "fastuidraw/gl_backend/glyph_atlas_gl.hpp"
#include "fastuidraw/gl_backend/painter_backend_gl.hpp"

extern fastuidraw::reference_counted_ptr<fastuidraw::gl::ImageAtlasGL> fui_imageAtlas;
extern fastuidraw::reference_counted_ptr<fastuidraw::gl::ColorStopAtlasGL> fui_colorStopAtlas;
extern fastuidraw::reference_counted_ptr<fastuidraw::gl::GlyphAtlasGL> fui_glyphAtlas;
