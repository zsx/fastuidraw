#pragma once

#ifdef __cplusplus
#include "fastuidraw/painter/packing/painter_backend.hpp"

typedef class fastuidraw::PainterBackend::Surface::Viewport fui_surface_viewport_t;
extern "C" {
#else
typedef struct Viewport fui_surface_viewport_t;

#endif

fui_surface_viewport_t *fui_surface_viewport_new(int x, int y, int w, int h);
void fui_surface_viewport_free(fui_surface_viewport_t *p);

void fui_surface_viewport_set_dimension(fui_surface_viewport_t *vwp, int w, int h);
void fui_surface_viewport_set_origin(fui_surface_viewport_t *vwp, int x, int y);
void fui_surface_viewport_get_dimension(fui_surface_viewport_t *vwp, int *w, int *h);
void fui_surface_viewport_get_origin(fui_surface_viewport_t *vwp, int *x, int *y);

#ifdef __cplusplus
}
#endif
