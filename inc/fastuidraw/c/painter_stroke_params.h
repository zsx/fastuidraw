#pragma once

#ifdef __cplusplus
#include "fastuidraw/painter/painter_stroke_params.hpp"

typedef class fastuidraw::PainterStrokeParams fui_painter_stroke_params_t;

extern "C" {
#else
typedef struct PainterStrokeParams fui_painter_stroke_params_t;
#endif


fui_painter_stroke_params_t *fui_painter_stroke_params_new();
void fui_painter_stroke_params_free(fui_painter_stroke_params_t *);

fui_painter_stroke_params_t *fui_painter_stroke_params_set_miter_limit(fui_painter_stroke_params_t *, float);
float fui_painter_stroke_params_get_miter_limit(const fui_painter_stroke_params_t *);

fui_painter_stroke_params_t *fui_painter_stroke_params_set_radius(fui_painter_stroke_params_t *, float);
float fui_painter_stroke_params_get_radius(const fui_painter_stroke_params_t *);

fui_painter_stroke_params_t *fui_painter_stroke_params_set_width(fui_painter_stroke_params_t *, float);
float fui_painter_stroke_params_get_width(const fui_painter_stroke_params_t *);

#ifdef __cplusplus
}
#endif
