#pragma once

#ifdef __cplusplus
extern "C" {
#endif

typedef struct PainterAttributeData fui_painter_attribute_data_t;
typedef struct PainterAttributeDataFilter fui_painter_attribute_data_filter_t;

fui_painter_attribute_data_t *fui_painter_attribute_data_new();

void fui_painter_attribute_data_set_data(fui_painter_attribute_data_t *, painter_attribute_data_filter_t *);
int fui_painter_attribute_data_get_index_adjust_chunk(const fui_painter_attribute_data_t *, unsigned int i);
int *fui_painter_attribute_data_get_index_adjust_chunks(const fui_painter_attribute_data_t *, size_t *len);

#ifdef __cplusplus
}
#endif
