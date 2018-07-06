#include "fastuidraw/c/painter_backend.h"
#include "fastuidraw/c/painter_backend_gl.h"


fui_gl_painter_backend_gl_t *fui_gl_painter_backend_gl_new(fui_gl_configuration_gl_t *conf)
{
    auto p = FASTUIDRAWnew fastuidraw::gl::PainterBackendGL(*conf, fastuidraw::PainterBackend::ConfigurationBase());
    //p->add_reference(p);
    return p;
}
void fui_gl_painter_backend_gl_free(fui_gl_painter_backend_gl_t *p)
{
    FASTUIDRAWdelete(p);
    //p->remove_reference(p);
}

fui_gl_surface_gl_properties_t *fui_gl_surface_gl_properties_new()
{
    return FASTUIDRAWnew fui_gl_surface_gl_properties_t();
}

void fui_gl_surface_gl_properties_free(fui_gl_surface_gl_properties_t *p)
{
    FASTUIDRAWdelete(p);
}

fui_gl_surface_gl_properties_t *fui_gl_surface_gl_properties_set_dimensions_xy(fui_gl_surface_gl_properties_t *props, int x, int y)
{
    return &props->dimensions(ivec2(x, y));
}

fui_gl_surface_gl_properties_t *fui_gl_surface_gl_properties_set_msaa(fui_gl_surface_gl_properties_t *props, int msaa)
{
    return &props->msaa(msaa);
}

int fui_gl_surface_gl_properties_get_msaa(const fui_gl_surface_gl_properties_t *props)
{
    return props->msaa();
}

fui_gl_surface_gl_t *fui_gl_surface_gl_new(fui_gl_surface_gl_properties_t *props)
{
    auto p = FASTUIDRAWnew fui_gl_surface_gl_t(*props);
    //p->add_reference(p);
    return p;
}

void fui_gl_surface_gl_free(fui_gl_surface_gl_t *p)
{
    FASTUIDRAWdelete(p);
    //p->remove_reference(p);
}

void fui_gl_surface_gl_blit_surface(fui_gl_surface_gl_t *p)
{
    p->blit_surface();
}

void fui_gl_surface_gl_clear_color_rgba(fui_gl_surface_gl_t *p, float r, float g, float b, float a)
{
    p->clear_color(vec4(r, g, b, a));
}

void fui_gl_surface_gl_clear_color(fui_gl_surface_gl_t *p)
{
    p->clear_color();
}

void fui_gl_surface_gl_set_viewport(fui_gl_surface_gl_t *p, fui_surface_viewport_t *vwp)
{
    p->viewport(*vwp);
}

