#include "fastuidraw/c/painter_dashed_stroke_params.h"

fui_painter_dashed_stroke_params_t *fui_painter_dashed_stroke_params_new()
{
    return FASTUIDRAWnew fastuidraw::PainterDashedStrokeParams();
}

void fui_painter_dashed_stroke_params_free(fui_painter_dashed_stroke_params_t *params)
{
    FASTUIDRAWdelete(params);
}

fui_painter_dashed_stroke_params_t *fui_painter_dashed_stroke_params_set_miter_limit(fui_painter_dashed_stroke_params_t *params, float f)
{
    return &params->miter_limit(f);
}

float fui_painter_dashed_stroke_params_get_miter_limit(const fui_painter_dashed_stroke_params_t *params)
{
    return params->miter_limit();
}

fui_painter_dashed_stroke_params_t *fui_painter_dashed_stroke_params_set_radius(fui_painter_dashed_stroke_params_t *params, float f)
{
    return &params->radius(f);
}
float fui_painter_dashed_stroke_params_get_radius(const fui_painter_dashed_stroke_params_t *params)
{
    return params->radius();
}

fui_painter_dashed_stroke_params_t *fui_painter_dashed_stroke_params_set_width(fui_painter_dashed_stroke_params_t *params, float f)
{
    return &params->width(f);
}
float fui_painter_dashed_stroke_params_get_width(const fui_painter_dashed_stroke_params_t *params)
{
    return params->width();
}

fui_painter_dashed_stroke_params_t *fui_painter_dashed_stroke_params_set_dash_offset(fui_painter_dashed_stroke_params_t *params, float f)
{
    return &params->dash_offset(f);
}
float fui_painter_dashed_stroke_params_get_dash_offset(const fui_painter_dashed_stroke_params_t *params)
{
    return params->dash_offset();
}

fui_painter_dashed_stroke_params_t *fui_painter_dashed_stroke_params_set_dash_pattern(fui_painter_dashed_stroke_params_t *params, float pattern[][2], size_t n_patterns)
{
    auto p_data = FASTUIDRAWnew fastuidraw::PainterDashedStrokeParams::DashPatternElement [n_patterns];
    for(size_t i = 0; i < n_patterns; i ++) {
        p_data[i].m_draw_length = pattern[i][0];
        p_data[i].m_space_length = pattern[i][1];
    }
    auto ret = &params->dash_pattern(c_array<const fastuidraw::PainterDashedStrokeParams::DashPatternElement>(p_data, n_patterns));
    FASTUIDRAWdelete_array(p_data);

    return ret;
}
