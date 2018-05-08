#pragma once

#ifdef __cplusplus
#include "fastuidraw/painter/painter_dashed_stroke_params.hpp"

using namespace fastuidraw;

extern "C" {
#else
#include <stddef.h> // for size_t
#endif

typedef struct PainterDashedStrokeParams fui_painter_dashed_stroke_params_t;

fui_painter_dashed_stroke_params_t *fui_painter_dashed_stroke_params_new();
void fui_painter_dashed_stroke_params_free(fui_painter_dashed_stroke_params_t *);

fui_painter_dashed_stroke_params_t *fui_painter_dashed_stroke_params_set_miter_limit(fui_painter_dashed_stroke_params_t *, float);
float fui_painter_dashed_stroke_params_get_miter_limit(const fui_painter_dashed_stroke_params_t *);

fui_painter_dashed_stroke_params_t *fui_painter_dashed_stroke_params_set_radius(fui_painter_dashed_stroke_params_t *, float);
float fui_painter_dashed_stroke_params_get_radius(const fui_painter_dashed_stroke_params_t *);

fui_painter_dashed_stroke_params_t *fui_painter_dashed_stroke_params_set_width(fui_painter_dashed_stroke_params_t *, float);
float fui_painter_dashed_stroke_params_get_width(const fui_painter_dashed_stroke_params_t *);

fui_painter_dashed_stroke_params_t *fui_painter_dashed_stroke_params_set_dash_offset(fui_painter_dashed_stroke_params_t *, float);
float fui_painter_dashed_stroke_params_get_dash_offset(const fui_painter_dashed_stroke_params_t *);

fui_painter_dashed_stroke_params_t *fui_painter_dashed_stroke_params_set_dash_pattern(fui_painter_dashed_stroke_params_t *, float pattern[][2], size_t n_patterns);

#ifdef __cplusplus
}
#endif
