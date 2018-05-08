#include "fastuidraw/c/colorstop_atlas_gl_params.h"

fui_gl_colorstop_atlas_gl_params_t *fui_gl_colorstop_atlas_gl_params_new()
{
    return FASTUIDRAWnew ColorStopAtlasGL::params();
}
void fui_gl_colorstop_atlas_gl_params_free(fui_gl_colorstop_atlas_gl_params_t *p)
{
    FASTUIDRAWdelete(p);
}
