#include "fastuidraw/c/glyph_atlas_gl.h"

fui_gl_glyph_atlas_gl_t *fui_gl_glyph_atlas_gl_new(fui_gl_glyph_atlas_gl_params_t *params)
{
    auto p = FASTUIDRAWnew GlyphAtlasGL(*params);
    p->add_reference(p);
    return p;
}

void fui_gl_glyph_atlas_gl_free(fui_gl_glyph_atlas_gl_t *p)
{
    p->remove_reference(p);
}

unsigned fui_gl_glyph_atlas_gl_geometry_texture(const fui_gl_glyph_atlas_gl_t *atlas)
{
    return atlas->geometry_texture();
}
