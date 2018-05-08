#pragma once

#include "fastuidraw/c/image_atlas_gl_params.h"
#ifdef __cplusplus
#include "fastuidraw/gl_backend/image_gl.hpp"

using namespace fastuidraw::gl;

extern "C" {
#endif

typedef struct ImageAtlasGL fui_gl_image_atlas_gl_t;

fui_gl_image_atlas_gl_t *fui_gl_image_atlas_gl_new(fui_gl_image_atlas_gl_params_t *params);
void fui_gl_image_atlas_gl_free(fui_gl_image_atlas_gl_t *p);

unsigned fui_gl_image_atlas_gl_color_texture(const fui_gl_image_atlas_gl_t *atlas);
unsigned fui_gl_image_atlas_gl_index_texture(const fui_gl_image_atlas_gl_t *atlas);

#ifdef __cplusplus
}
#endif
