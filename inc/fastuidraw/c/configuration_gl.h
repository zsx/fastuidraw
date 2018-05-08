#pragma once

#include "fastuidraw/c/colorstop_atlas_gl.h"
#include "fastuidraw/c/glyph_atlas_gl.h"
#include "fastuidraw/c/image_atlas_gl.h"

#ifdef __cplusplus
#include "fastuidraw/gl_backend/painter_backend_gl.hpp"

using namespace fastuidraw::gl;
typedef struct PainterBackendGL::ConfigurationGL fui_gl_configuration_gl_t;

extern "C" {
#else
typedef struct ConfigurationGL fui_gl_configuration_gl_t;
#endif

fui_gl_configuration_gl_t *fui_gl_configuration_gl_new();
void fui_gl_configuration_gl_free(fui_gl_configuration_gl_t *p);

fui_gl_configuration_gl_t
    *fui_gl_configuration_gl_set_glyph_atlas(fui_gl_configuration_gl_t *conf,
                                             fui_gl_glyph_atlas_gl_t *glyph);

fui_gl_configuration_gl_t
    *fui_gl_configuration_gl_set_image_atlas(fui_gl_configuration_gl_t *conf,
                                             fui_gl_image_atlas_gl_t *image);

fui_gl_configuration_gl_t
    *fui_gl_configuration_gl_set_colorstop_atlas(fui_gl_configuration_gl_t
                                                 *conf,
                                                 fui_gl_colorstop_atlas_gl_t
                                                 *colorstop);

fui_gl_glyph_atlas_gl_t *fui_gl_configuration_gl_get_glyph_atlas(const fui_gl_configuration_gl_t *conf);
fui_gl_image_atlas_gl_t *fui_gl_configuration_gl_get_image_atlas(const fui_gl_configuration_gl_t *conf);
fui_gl_colorstop_atlas_gl_t *fui_gl_configuration_gl_get_colorstop_atlas(const fui_gl_configuration_gl_t *conf);

fui_gl_configuration_gl_t *fui_gl_configuration_gl_set_assign_binding_points(fui_gl_configuration_gl_t *conf, int v);
int fui_gl_configuration_gl_get_assign_layout_to_varyings(const fui_gl_configuration_gl_t *conf);

fui_gl_configuration_gl_t *fui_gl_configuration_gl_set_assign_layout_to_varyings(fui_gl_configuration_gl_t *conf, int v);
int fui_gl_configuration_gl_get_assign_layout_to_vertex_shader_inputs(const fui_gl_configuration_gl_t *conf);

fui_gl_configuration_gl_t *fui_gl_configuration_gl_set_assign_layout_to_vertex_shader_inputs(fui_gl_configuration_gl_t *conf, int v);

int fui_gl_configuration_gl_get_attributes_per_buffer(const fui_gl_configuration_gl_t *conf);

fui_gl_configuration_gl_t *fui_gl_configuration_gl_set_attributes_per_buffer(fui_gl_configuration_gl_t *conf, int v);

int fui_gl_configuration_gl_get_blend_shader_use_switch(const fui_gl_configuration_gl_t *conf);

fui_gl_configuration_gl_t *fui_gl_configuration_gl_set_blend_shader_use_switch(fui_gl_configuration_gl_t *conf, int v);

int fui_gl_configuration_gl_get_break_on_shader_change(const fui_gl_configuration_gl_t *conf);

fui_gl_configuration_gl_t *fui_gl_configuration_gl_set_break_on_shader_change(fui_gl_configuration_gl_t *conf, int v);

int fui_gl_configuration_gl_get_data_blocks_per_store_buffer(const fui_gl_configuration_gl_t *conf);

fui_gl_configuration_gl_t *fui_gl_configuration_gl_set_data_blocks_per_store_buffer(fui_gl_configuration_gl_t *conf, int v);

int fui_gl_configuration_gl_get_frag_shader_use_switch(const fui_gl_configuration_gl_t *conf);

fui_gl_configuration_gl_t *fui_gl_configuration_gl_set_frag_shader_use_switch(fui_gl_configuration_gl_t *conf, int v);

int fui_gl_configuration_gl_get_indices_per_buffer(const fui_gl_configuration_gl_t *conf);

fui_gl_configuration_gl_t *fui_gl_configuration_gl_set_indices_per_buffer(fui_gl_configuration_gl_t *conf, int v);

int fui_gl_configuration_gl_get_non_dashed_stroke_shader_uses_discard(const fui_gl_configuration_gl_t *conf);

fui_gl_configuration_gl_t *fui_gl_configuration_gl_set_non_dashed_stroke_shader_uses_discard(fui_gl_configuration_gl_t *conf, int v);

int fui_gl_configuration_gl_get_number_pools(const fui_gl_configuration_gl_t *conf);

fui_gl_configuration_gl_t *fui_gl_configuration_gl_set_number_pools(fui_gl_configuration_gl_t *conf, int v);

int fui_gl_configuration_gl_get_separate_program_for_discard(const fui_gl_configuration_gl_t *conf);

fui_gl_configuration_gl_t *fui_gl_configuration_gl_set_separate_program_for_discard(fui_gl_configuration_gl_t *conf, int v);

int fui_gl_configuration_gl_get_unpack_header_and_brush_in_frag_shader(const fui_gl_configuration_gl_t *conf);

fui_gl_configuration_gl_t *fui_gl_configuration_gl_set_unpack_header_and_brush_in_frag_shader(fui_gl_configuration_gl_t *conf, int v);
int fui_gl_configuration_gl_get_use_hw_clip_planes(const fui_gl_configuration_gl_t *conf);

fui_gl_configuration_gl_t *fui_gl_configuration_gl_set_use_hw_clip_planes(fui_gl_configuration_gl_t *conf, int v);

int fui_gl_configuration_gl_get_use_ubo_for_uniforms(const fui_gl_configuration_gl_t *conf);
fui_gl_configuration_gl_t *fui_gl_configuration_gl_set_use_ubo_for_uniforms(fui_gl_configuration_gl_t *conf, int v);

int fui_gl_configuration_gl_get_vert_shader_use_switch(const fui_gl_configuration_gl_t *conf);

fui_gl_configuration_gl_t *fui_gl_configuration_gl_set_vert_shader_user_switch(fui_gl_configuration_gl_t *conf, int v);

#ifdef __cplusplus
}
#endif
