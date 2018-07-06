#pragma once

#ifdef __cplusplus
#include "fastuidraw/gl_backend/image_gl.hpp"

using namespace fastuidraw::gl;

typedef class ImageAtlasGL::params fui_gl_image_atlas_gl_params_t;
extern "C" {
#else
typedef struct params fui_gl_image_atlas_gl_params_t;
#endif

fui_gl_image_atlas_gl_params_t *fui_gl_image_atlas_gl_params_new();
void fui_gl_image_atlas_gl_params_free(fui_gl_image_atlas_gl_params_t *p);

#ifdef __cplusplus
}
#endif
