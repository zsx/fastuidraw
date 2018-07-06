#include "fastuidraw/c/painter_backend.h"

fui_surface_viewport_t *
fui_surface_viewport_new(int x, int y, int w, int h)
{
    return FASTUIDRAWnew fui_surface_viewport_t(x, y, w, h);
}

void fui_surface_viewport_free(fui_surface_viewport_t *p)
{
    FASTUIDRAWdelete(p);
}

void fui_surface_viewport_set_dimension(fui_surface_viewport_t *vwp, int w, int h)
{
    vwp->m_dimensions = fastuidraw::ivec2(w, h);
}

void fui_surface_viewport_set_origin(fui_surface_viewport_t *vwp, int x, int y)
{
    vwp->m_origin = fastuidraw::ivec2(x, y);
}

void fui_surface_viewport_get_dimension(fui_surface_viewport_t *vwp, int *w, int *h)
{
    if (w) *w = vwp->m_dimensions[0];
    if (h) *h = vwp->m_dimensions[1];
}

void fui_surface_viewport_get_origin(fui_surface_viewport_t *vwp, int *x, int *y)
{
    if (x) *x = vwp->m_origin[0];
    if (y) *y = vwp->m_origin[1];
}

