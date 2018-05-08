#include "fastuidraw/c/configuration_gl.h"
#include "fastuidraw/util/reference_counted.hpp"

using fastuidraw::reference_counted_ptr;

fui_gl_configuration_gl_t *fui_gl_configuration_gl_new()
{
    return FASTUIDRAWnew PainterBackendGL::ConfigurationGL();
}
void fui_gl_configuration_gl_free(fui_gl_configuration_gl_t *p)
{
    FASTUIDRAWdelete(p);
}

fui_gl_configuration_gl_t
    *fui_gl_configuration_gl_set_glyph_atlas(fui_gl_configuration_gl_t *conf,
                                             fui_gl_glyph_atlas_gl_t *glyph)
{
    reference_counted_ptr<GlyphAtlasGL> rc(glyph);
    return & conf->glyph_atlas(rc);
}

fui_gl_configuration_gl_t
    *fui_gl_configuration_gl_set_image_atlas(fui_gl_configuration_gl_t *conf,
                                             fui_gl_image_atlas_gl_t *image)
{
    reference_counted_ptr<ImageAtlasGL> rc(image);
    auto ret = &conf->image_atlas(rc);
    return ret;
}

fui_gl_configuration_gl_t
    *fui_gl_configuration_gl_set_colorstop_atlas(fui_gl_configuration_gl_t
                                                 *conf,
                                                 fui_gl_colorstop_atlas_gl_t
                                                 *colorstop)
{
    reference_counted_ptr<ColorStopAtlasGL> rc(colorstop);
    return & conf->colorstop_atlas(rc);
}

fui_gl_glyph_atlas_gl_t *fui_gl_configuration_gl_get_glyph_atlas(const fui_gl_configuration_gl_t *conf)
{
    return conf->glyph_atlas().get();
}

fui_gl_image_atlas_gl_t *fui_gl_configuration_gl_get_image_atlas(const fui_gl_configuration_gl_t *conf)
{
    return conf->image_atlas().get();
}

fui_gl_colorstop_atlas_gl_t *fui_gl_configuration_gl_get_colorstop_atlas(const fui_gl_configuration_gl_t *conf)
{
    return conf->colorstop_atlas().get();
}

int fui_gl_configuration_gl_get_assign_binding_points(const fui_gl_configuration_gl_t *conf)
{
    return conf->assign_binding_points();
}

fui_gl_configuration_gl_t *fui_gl_configuration_gl_set_assign_binding_points(fui_gl_configuration_gl_t *conf, int v)
{
    return & conf->assign_binding_points(v);
}

int fui_gl_configuration_gl_get_assign_layout_to_varyings(const fui_gl_configuration_gl_t *conf)
{
    return conf->assign_layout_to_varyings();
}

fui_gl_configuration_gl_t *fui_gl_configuration_gl_set_assign_layout_to_varyings(fui_gl_configuration_gl_t *conf, int v)
{
    return & conf->assign_layout_to_varyings(v);
}

int fui_gl_configuration_gl_get_assign_layout_to_vertex_shader_inputs(const fui_gl_configuration_gl_t *conf)
{
    return conf->assign_layout_to_vertex_shader_inputs();
}

fui_gl_configuration_gl_t *fui_gl_configuration_gl_set_assign_layout_to_vertex_shader_inputs(fui_gl_configuration_gl_t *conf, int v)
{
    return & conf->assign_layout_to_vertex_shader_inputs(v);
}

int fui_gl_configuration_gl_get_attributes_per_buffer(const fui_gl_configuration_gl_t *conf)
{
    return conf->attributes_per_buffer();
}

fui_gl_configuration_gl_t *fui_gl_configuration_gl_set_attributes_per_buffer(fui_gl_configuration_gl_t *conf, int v)
{
    return & conf->attributes_per_buffer(v);
}

int fui_gl_configuration_gl_get_blend_shader_use_switch(const fui_gl_configuration_gl_t *conf)
{
    return conf->blend_shader_use_switch();
}

fui_gl_configuration_gl_t *fui_gl_configuration_gl_set_blend_shader_use_switch(fui_gl_configuration_gl_t *conf, int v)
{
    return & conf->blend_shader_use_switch(v);
}

int fui_gl_configuration_gl_get_break_on_shader_change(const fui_gl_configuration_gl_t *conf)
{
    return conf->break_on_shader_change();
}

fui_gl_configuration_gl_t *fui_gl_configuration_gl_set_break_on_shader_change(fui_gl_configuration_gl_t *conf, int v)
{
    return & conf->break_on_shader_change(v);
}

int fui_gl_configuration_gl_get_data_blocks_per_store_buffer(const fui_gl_configuration_gl_t *conf)
{
    return conf->data_blocks_per_store_buffer();
}

fui_gl_configuration_gl_t *fui_gl_configuration_gl_set_data_blocks_per_store_buffer(fui_gl_configuration_gl_t *conf, int v)
{
    return & conf->data_blocks_per_store_buffer(v);
}

int fui_gl_configuration_gl_get_frag_shader_use_switch(const fui_gl_configuration_gl_t *conf)
{
    return conf->frag_shader_use_switch();
}

fui_gl_configuration_gl_t *fui_gl_configuration_gl_set_frag_shader_use_switch(fui_gl_configuration_gl_t *conf, int v)
{
    return & conf->frag_shader_use_switch(v);
}

int fui_gl_configuration_gl_get_indices_per_buffer(const fui_gl_configuration_gl_t *conf)
{
    return conf->indices_per_buffer();
}

fui_gl_configuration_gl_t *fui_gl_configuration_gl_set_indices_per_buffer(fui_gl_configuration_gl_t *conf, int v)
{
    return & conf->indices_per_buffer(v);
}

#if 0
int fui_gl_configuration_gl_get_non_dashed_stroke_shader_uses_discard(const fui_gl_configuration_gl_t *conf)
{
    return conf->non_dashed_stroke_shader_uses_discard();
}

fui_gl_configuration_gl_t *fui_gl_configuration_gl_set_non_dashed_stroke_shader_uses_discard(fui_gl_configuration_gl_t *conf, int v)
{
    return & conf->non_dashed_stroke_shader_uses_discard(v);
}
#endif

int fui_gl_configuration_gl_get_number_pools(const fui_gl_configuration_gl_t *conf)
{
    return conf->number_pools();
}

fui_gl_configuration_gl_t *fui_gl_configuration_gl_set_number_pools(fui_gl_configuration_gl_t *conf, int v)
{
    return & conf->number_pools(v);
}

int fui_gl_configuration_gl_get_separate_program_for_discard(const fui_gl_configuration_gl_t *conf)
{
    return conf->separate_program_for_discard();
}

fui_gl_configuration_gl_t *fui_gl_configuration_gl_set_separate_program_for_discard(fui_gl_configuration_gl_t *conf, int v)
{
    return & conf->separate_program_for_discard(v);
}

int fui_gl_configuration_gl_get_unpack_header_and_brush_in_frag_shader(const fui_gl_configuration_gl_t *conf)
{
    return conf->unpack_header_and_brush_in_frag_shader();
}

fui_gl_configuration_gl_t *fui_gl_configuration_gl_set_unpack_header_and_brush_in_frag_shader(fui_gl_configuration_gl_t *conf, int v)
{
    return & conf->unpack_header_and_brush_in_frag_shader(v);
}

int fui_gl_configuration_gl_get_use_hw_clip_planes(const fui_gl_configuration_gl_t *conf)
{
    return conf->use_hw_clip_planes();
}

fui_gl_configuration_gl_t *fui_gl_configuration_gl_set_use_hw_clip_planes(fui_gl_configuration_gl_t *conf, int v)
{
    return & conf->use_hw_clip_planes(v);
}

#if 0
int fui_gl_configuration_gl_get_use_ubo_for_uniforms(const fui_gl_configuration_gl_t *conf)
{
    return conf->use_ubo_for_uniforms();
}
fui_gl_configuration_gl_t *fui_gl_configuration_gl_set_use_ubo_for_uniforms(fui_gl_configuration_gl_t *conf, int v)
{
    return & conf->use_ubo_for_uniforms(v);
}

#endif

int fui_gl_configuration_gl_get_vert_shader_use_switch(const fui_gl_configuration_gl_t *conf)
{
    return conf->vert_shader_use_switch();
}

fui_gl_configuration_gl_t *fui_gl_configuration_gl_set_vert_shader_user_switch(fui_gl_configuration_gl_t *conf, int v)
{
    return & conf->vert_shader_use_switch(v);
}
