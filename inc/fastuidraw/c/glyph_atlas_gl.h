#pragma once

#include "fastuidraw/c/glyph_atlas_gl_params.h"
#ifdef __cplusplus
#include "fastuidraw/gl_backend/glyph_atlas_gl.hpp"

using namespace fastuidraw::gl;
typedef class GlyphAtlasGL fui_gl_glyph_atlas_gl_t;

extern "C" {
#else
typedef struct GlyphAtlasGL fui_gl_glyph_atlas_gl_t;
#endif


fui_gl_glyph_atlas_gl_t *fui_gl_glyph_atlas_gl_new(fui_gl_glyph_atlas_gl_params_t *params);
void fui_gl_glyph_atlas_gl_free(fui_gl_glyph_atlas_gl_t *p);

unsigned fui_gl_glyph_atlas_gl_geometry_texture(const fui_gl_glyph_atlas_gl_t *atlas);

#ifdef __cplusplus
}
#endif
