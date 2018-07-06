#include "fastuidraw/c/colorstop_atlas_gl.h"

fui_gl_colorstop_atlas_gl_t *fui_gl_colorstop_atlas_gl_new(fui_gl_colorstop_atlas_gl_params_t *params)
{
    auto p = FASTUIDRAWnew ColorStopAtlasGL(*params);
    //p->add_reference(p);
    return p;
}

void fui_gl_colorstop_atlas_gl_free(fui_gl_colorstop_atlas_gl_t *p)
{
    FASTUIDRAWdelete(p);
    //p->remove_reference(p);
}

unsigned fui_gl_colorstop_atlas_gl_texture(const fui_gl_colorstop_atlas_gl_t *atlas)
{
    return atlas->texture();
}

fui_colorstop_sequence_t *fui_colorstop_sequence_new()
{
    return FASTUIDRAWnew ColorStopSequence();
}
void fui_colorstop_sequence_free(fui_colorstop_sequence_t *p)
{
    FASTUIDRAWdelete(p);
}

void fui_colorstop_sequence_add_components(fui_colorstop_sequence_t *colorstop_sequence,
                                           uint8_t r, uint8_t g, uint8_t b, uint8_t a, float place)
{
    colorstop_sequence->add(fastuidraw::ColorStop(fastuidraw::u8vec4(r, g, b, a), place));
}
void fui_colorstop_sequence_clear(fui_colorstop_sequence_t *colorstop_sequence)
{
    colorstop_sequence->clear();
}

fui_colorstop_sequence_on_atlas_t *fui_colorstop_sequence_on_atlas_new_gl(fui_colorstop_sequence_t *seq, fui_gl_colorstop_atlas_gl_t *atlas, int pwidth)
{
    auto p = FASTUIDRAWnew ColorStopSequenceOnAtlas(*seq,
                                                    fastuidraw::reference_counted_ptr<fastuidraw::ColorStopAtlas>(atlas),
                                                    pwidth);
    p->add_reference(p);
    return p;
}
void fui_colorstop_sequence_on_atlas_free(fui_colorstop_sequence_on_atlas_t *p)
{
    p->remove_reference(p);
}
