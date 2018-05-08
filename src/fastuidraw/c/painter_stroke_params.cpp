#include "fastuidraw/c/painter_stroke_params.h"

fui_painter_stroke_params_t *fui_painter_stroke_params_new()
{
    return FASTUIDRAWnew fastuidraw::PainterStrokeParams();
}

void fui_painter_stroke_params_free(fui_painter_stroke_params_t *params)
{
    FASTUIDRAWdelete(params);
}

fui_painter_stroke_params_t *fui_painter_stroke_params_set_miter_limit(fui_painter_stroke_params_t *params, float f)
{
    return &params->miter_limit(f);
}

float fui_painter_stroke_params_get_miter_limit(const fui_painter_stroke_params_t *params)
{
    return params->miter_limit();
}

fui_painter_stroke_params_t *fui_painter_stroke_params_set_radius(fui_painter_stroke_params_t *params, float f)
{
    return &params->radius(f);
}
float fui_painter_stroke_params_get_radius(const fui_painter_stroke_params_t *params)
{
    return params->radius();
}

fui_painter_stroke_params_t *fui_painter_stroke_params_set_width(fui_painter_stroke_params_t *params, float f)
{
    return &params->width(f);
}
float fui_painter_stroke_params_get_width(const fui_painter_stroke_params_t *params)
{
    return params->width();
}

