#include "fastuidraw/c/image_atlas_gl_params.h"

fui_gl_image_atlas_gl_params_t *fui_gl_image_atlas_gl_params_new()
{
    return FASTUIDRAWnew ImageAtlasGL::params();
}
void fui_gl_image_atlas_gl_params_free(fui_gl_image_atlas_gl_params_t *p)
{
    FASTUIDRAWdelete(p);
}
