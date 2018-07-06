#pragma once

#include "fastuidraw/c/painter_backend_gl.h"
#include "fastuidraw/c/painter_dashed_stroke_params.h"
#include "fastuidraw/c/painter_stroke_params.h"
#include "fastuidraw/c/painter_brush.h"
#include "fastuidraw/c/path.h"

#ifdef __cplusplus

#include "fastuidraw/painter/painter.hpp"

using namespace fastuidraw;
typedef class Painter fui_painter_t;
typedef class PainterData fui_painter_data_t;

extern "C" {
#else
typedef struct Painter fui_painter_t;
typedef struct PainterData fui_painter_data_t;
#endif

fui_painter_t *fui_painter_new_gl(fui_gl_painter_backend_gl_t *backend);
void fui_painter_free(fui_painter_t *);

int fui_painter_current_z(const fui_painter_t *painter);
void fui_painter_increment_z(fui_painter_t *painter, int amount);
void fui_painter_curveFlatness_set(fui_painter_t *painter, float thresh);
float fui_painter_curveFlatness_get(fui_painter_t *painter);
void fui_painter_fill_path(fui_painter_t *painter,
                           const fui_painter_brush_t *brush,
                           const fui_path_t *,
                           int fill_rule,
                           int with_anti_aliasing);

//void fui_painter_set_target_resolution(fui_painter_t *, int w, int h);
void fui_painter_begin(fui_painter_t *painter, fui_gl_surface_gl_t *surface, int reset_z);

void fui_painter_end(fui_painter_t *);
void fui_painter_restore(fui_painter_t *);
void fui_painter_rotate(fui_painter_t *, float angle);
void fui_painter_save(fui_painter_t *);
void fui_painter_scale(fui_painter_t *, float s);
void fui_painter_shear(fui_painter_t *, float sx, float sy);

void fui_painter_set_transformation_3x3(fui_painter_t *painter, const float m[3][3]);
void fui_painter_get_transformation_3x3(fui_painter_t *painter, float m[3][3]);

void fui_painter_translate(fui_painter_t *painter, const float p[2]);
void fui_painter_translate_xy(fui_painter_t *painter, float x, float y);

void fui_painter_clipInRect(fui_painter_t *painter, const float xy[2], const float wh[2]);

void fui_painter_clipInPath(fui_painter_t *painter, const fui_path_t *, int fill_rule);
void fui_painter_clipOutPath(fui_painter_t *painter, const fui_path_t *, int fill_rule);

void fui_painter_stroke_path(fui_painter_t *painter,
                             const fui_path_t *path,
                             const fui_painter_brush_t *brush,
                             const fui_painter_stroke_params_t *stroke_params,
                             int close_contours,
                             int cap,
                             int join_style,
                             int anti_aliasing);

void fui_painter_stroke_dashed_path(fui_painter_t *painter,
                                    const fui_path_t *path,
                                    const fui_painter_brush_t *brush,
                                    const fui_painter_dashed_stroke_params_t *stroke_params,
                                    int close_contours,
                                    int cap,
                                    int join_style,
                                    int anti_aliasing);

void fui_painter_draw_rect(fui_painter_t *painter,
                           fui_painter_brush_t *brush,
                           const float p[2],
                           const float wh[2]);

#if 0
void fui_painter_draw_convex_polygon_default_shader(fui_painter_t *, fui_painter_data_t *data, const float[2] *pts, size_t n_points, int anti_aliasing);
void fui_painter_draw_glyph_default_shader(fui_painter_t *, fui_painter_data_t *data, fui_painter_attribute_data_t *, int anti_aliasing);
void fui_painter_draw_quad_default_shader(fui_painter_t *, fui_painter_data_t *data, const float[2] p0, const float[2] p1, const float[2] p2, const float[2] p3, int anti_aliasing);

void fui_path_stroke_dashed_path_default_shader(const fui_painter_data_t *,
                                                const path_t *,
                                                int close_contours,
                                                int cap,
                                                int join_style,
                                                int anti_aliasing);


#endif

#ifdef __cplusplus
}
#endif
