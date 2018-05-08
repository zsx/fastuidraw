#include "fastuidraw/c/glyph_atlas_gl_params.h"

fui_gl_glyph_atlas_gl_params_t *fui_gl_glyph_atlas_gl_params_new()
{
    return FASTUIDRAWnew GlyphAtlasGL::params();
}
void fui_gl_glyph_atlas_gl_params_free(fui_gl_glyph_atlas_gl_params_t *p)
{
    FASTUIDRAWdelete(p);
}
