#pragma once

#ifdef __cplusplus
#include "fastuidraw/gl_backend/colorstop_atlas_gl.hpp"

using namespace fastuidraw::gl;

typedef class ColorStopAtlasGL::params fui_gl_colorstop_atlas_gl_params_t;
extern "C" {
#else
typedef struct params fui_gl_colorstop_atlas_gl_params_t;
#endif

fui_gl_colorstop_atlas_gl_params_t *fui_gl_colorstop_atlas_gl_params_new();
void fui_gl_colorstop_atlas_gl_params_free(fui_gl_colorstop_atlas_gl_params_t *p);

#ifdef __cplusplus
}
#endif
