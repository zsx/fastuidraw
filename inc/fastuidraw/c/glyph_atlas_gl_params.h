#pragma once

#ifdef __cplusplus
#include "fastuidraw/gl_backend/glyph_atlas_gl.hpp"

using namespace fastuidraw::gl;

typedef struct GlyphAtlasGL::params fui_gl_glyph_atlas_gl_params_t;
extern "C" {
#else
typedef struct glyph_atlas_params fui_gl_glyph_atlas_gl_params_t;
#endif

fui_gl_glyph_atlas_gl_params_t *fui_gl_glyph_atlas_gl_params_new();
void fui_gl_glyph_atlas_gl_params_free(fui_gl_glyph_atlas_gl_params_t *p);

#ifdef __cplusplus
}
#endif
