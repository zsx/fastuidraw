#pragma once

typedef struct PainterData fui_painter_data_t;

#ifdef __cplusplus
extern "C" {
#endif

fui_painter_data_t *fui_painter_data_new(fui_painter_data_backend_t *backend);
fui_painter_data_t *fui_painter_data_new_with_brush_stroke_params(fui_painter_brush_t *brush, fui_painter_stroke_params *);
void fui_painter_data_free(fui_painter_data_t *);


#ifdef __cplusplus
}
#endif
