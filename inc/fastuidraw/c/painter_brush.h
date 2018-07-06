#pragma once

#include "fastuidraw/c/colorstop_atlas_gl.h"
#include "fastuidraw/c/image.h"
#ifdef __cplusplus
#include "fastuidraw/painter/painter_brush.hpp"

using namespace fastuidraw;
typedef class PainterBrush fui_painter_brush_t;
extern "C" {
#else
typedef struct PainterBrush fui_painter_brush_t;
#endif

fui_painter_brush_t *fui_painter_brush_new();
fui_painter_brush_t *fui_painter_brush_new_with_pen(const float color[4]);
void fui_painter_brush_free(fui_painter_brush_t *brush);

fui_painter_brush_t *fui_painter_brush_set_image(fui_painter_brush_t *brush,
                                                 const fui_image_t *image,
                                                 int filter);
fui_painter_brush_t *fui_painter_brush_set_sub_image(fui_painter_brush_t *brush,
                                                     const fui_image_t *image,
                                                     uint32_t x, uint32_t y,
                                                     uint32_t w, uint32_t h,
                                                     int filter);
fui_painter_brush_t *fui_painter_brush_set_pen(fui_painter_brush_t *brush, const float color[4]);
fui_painter_brush_t *fui_painter_brush_set_pen_rgba(fui_painter_brush_t *brush, float r, float g, float b, float a);
fui_painter_brush_t *fui_painter_brush_set_transformation_translate_xy(fui_painter_brush_t *brush, float x, float y);
fui_painter_brush_t *fui_painter_brush_set_no_image(fui_painter_brush_t *brush);
fui_painter_brush_t *fui_painter_brush_set_no_gradient(fui_painter_brush_t *brush);
fui_painter_brush_t *fui_painter_brush_set_no_repeat_window(fui_painter_brush_t *brush);
fui_painter_brush_t *fui_painter_brush_set_no_transformation(fui_painter_brush_t *brush);
fui_painter_brush_t *fui_painter_brush_set_no_transformation_matrix(fui_painter_brush_t *brush);
fui_painter_brush_t *fui_painter_brush_set_no_transformation_translation(fui_painter_brush_t *brush);
void fui_painter_brush_reset(fui_painter_brush_t *brush);
fui_painter_brush_t *fui_painter_brush_set_linear_gradient(fui_painter_brush_t *brush,
                                                           fui_colorstop_sequence_on_atlas_t *seq,
                                                           float x1, float y1,
                                                           float x2, float y2,
                                                           int repeat);
fui_painter_brush_t *fui_painter_brush_set_radial_gradient(fui_painter_brush_t *brush,
                                                           fui_colorstop_sequence_on_atlas_t *seq,
                                                           float x1, float y1, float r1,
                                                           float x2, float y2, float r2,
                                                           int repeat);
fui_painter_brush_t *fui_painter_brush_set_repeat_window(fui_painter_brush_t *brush,
                                                         float x, float y,
                                                         float w, float h);

#ifdef __cplusplus
}
#endif
