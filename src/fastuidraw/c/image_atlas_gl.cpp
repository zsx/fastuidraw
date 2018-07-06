#include "fastuidraw/c/image_atlas_gl.h"

fui_gl_image_atlas_gl_t *fui_gl_image_atlas_gl_new(fui_gl_image_atlas_gl_params_t *params)
{
    auto p = FASTUIDRAWnew ImageAtlasGL(*params);
    //p->add_reference(p);
    return p;
}

void fui_gl_image_atlas_gl_free(fui_gl_image_atlas_gl_t *p)
{
    FASTUIDRAWdelete(p);
    //p->remove_reference(p);
}

unsigned fui_gl_image_atlas_gl_color_texture(const fui_gl_image_atlas_gl_t *atlas)
{
    return atlas->color_texture();
}

unsigned fui_gl_image_atlas_gl_index_texture(const fui_gl_image_atlas_gl_t *atlas)
{
    return atlas->index_texture();
}
