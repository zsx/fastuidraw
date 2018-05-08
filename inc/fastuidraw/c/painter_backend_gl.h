#pragma once

#include "fastuidraw/c/configuration_gl.h"

#ifdef __cplusplus
#include "fastuidraw/gl_backend/painter_backend_gl.hpp"

using namespace fastuidraw;
typedef struct gl::PainterBackendGL fui_gl_painter_backend_gl_t;
typedef struct gl::PainterBackendGL::SurfaceGL fui_gl_surface_gl_t;
typedef struct gl::PainterBackendGL::SurfaceGL::Properties fui_gl_surface_gl_properties_t;
extern "C" {
#else
typedef struct PainterBackendGL fui_gl_painter_backend_gl_t;
typedef struct SurfaceGL fui_gl_surface_gl_t;
typedef struct Properties fui_gl_surface_gl_properties_t;

#endif


fui_gl_painter_backend_gl_t *fui_gl_painter_backend_gl_new(fui_gl_configuration_gl_t *conf);
void fui_gl_painter_backend_gl_free(fui_gl_painter_backend_gl_t *p);

fui_gl_surface_gl_properties_t *fui_gl_surface_gl_properties_new();
void fui_gl_surface_gl_properties_free(fui_gl_surface_gl_properties_t *p);

fui_gl_surface_gl_properties_t *fui_gl_surface_gl_properties_set_dimensions_xy(fui_gl_surface_gl_properties_t *props, int x, int y);
fui_gl_surface_gl_properties_t *fui_gl_surface_gl_properties_set_msaa(fui_gl_surface_gl_properties_t *props, int msaa);
int fui_gl_surface_gl_properties_get_msaa(const fui_gl_surface_gl_properties_t *props);

fui_gl_surface_gl_t *fui_gl_surface_gl_new(fui_gl_surface_gl_properties_t *props);
void fui_gl_surface_gl_free(fui_gl_surface_gl_t *p);

void fui_gl_surface_gl_blit_surface(fui_gl_surface_gl_t *p);

#ifdef __cplusplus
}
#endif
