#pragma once

#include <inttypes.h>
#include "fastuidraw/c/colorstop_atlas_gl_params.h"
#ifdef __cplusplus
#include "fastuidraw/gl_backend/colorstop_atlas_gl.hpp"

using namespace fastuidraw::gl;
using fastuidraw::ColorStop;
using fastuidraw::ColorStopSequence;
using fastuidraw::ColorStopSequenceOnAtlas;

typedef class ColorStopAtlasGL fui_gl_colorstop_atlas_gl_t;
typedef class ColorStopSequence  fui_colorstop_sequence_t;
typedef class ColorStopSequenceOnAtlas fui_colorstop_sequence_on_atlas_t;

extern "C" {
#else
typedef struct ColorStopAtlasGL fui_gl_colorstop_atlas_gl_t;
typedef struct ColorStopSequence  fui_colorstop_sequence_t;
typedef struct ColorStopSequenceOnAtlas fui_colorstop_sequence_on_atlas_t;
#endif

fui_gl_colorstop_atlas_gl_t *fui_gl_colorstop_atlas_gl_new(fui_gl_colorstop_atlas_gl_params_t *params);
void fui_gl_colorstop_atlas_gl_free(fui_gl_colorstop_atlas_gl_t *p);

unsigned fui_gl_colorstop_atlas_gl_texture(const fui_gl_colorstop_atlas_gl_t *atlas);

/*
typedef struct ColorStop fui_colorstop_t;
fui_colorstop_t *fui_colorstop_new();
fui_colorstop_t *fui_colorstop_new_with_color_place(uint8_t r, uint8_t g, uint8_t b, uint8_t a, float place);
void fui_colorstop_free(fui_colorstop_t *colorstop);
*/

fui_colorstop_sequence_t *fui_colorstop_sequence_new();
void fui_colorstop_sequence_free(fui_colorstop_sequence_t *p);
//void fui_colorstop_sequence_add(fui_colorstop_sequence_t *colorstop_sequence, fui_colorstop_t *colorstop);
void fui_colorstop_sequence_add_components(fui_colorstop_sequence_t *colorstop_sequence,
                                           uint8_t r, uint8_t g, uint8_t b, uint8_t a, float place);
void fui_colorstop_sequence_clear(fui_colorstop_sequence_t *colorstop_sequence);

fui_colorstop_sequence_on_atlas_t *fui_colorstop_sequence_on_atlas_new_gl(fui_colorstop_sequence_t *seq, fui_gl_colorstop_atlas_gl_t *atlas, int pwidth);
void fui_colorstop_sequence_on_atlas_free(fui_colorstop_sequence_on_atlas_t *p);

#ifdef __cplusplus
}
#endif
