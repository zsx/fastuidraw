#pragma once

#include "fastuidraw/c/image_atlas_gl.h"

#ifdef __cplusplus

#include "fastuidraw/image.hpp"

using namespace fastuidraw;
typedef class Image fui_image_t;
typedef class ImageAtlas fui_image_atlas_t;

extern "C" {
#else
typedef struct Image fui_image_t;
typedef struct ImageAtlas fui_image_atlas_t;
#endif

fui_image_t *fui_image_new_from_RGBA8888(fui_image_atlas_t* atlas, int w, int h, int flip_y, const char *data, unsigned int pslack);
void fui_image_free(fui_image_t *image);

void fui_image_dimensions(const fui_image_t *image, int *w, int *h);
//float fui_image_dimensions_index_divisor(const fui_image_t *image);

#ifdef __cplusplus
}
#endif
