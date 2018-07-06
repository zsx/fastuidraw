#pragma once

#ifdef __cplusplus
#include "fastuidraw/path.hpp"

using namespace fastuidraw;

typedef class Path fui_path_t;

extern "C" {
#else
typedef struct Path fui_path_t;
#endif


fui_path_t *fui_path_new();
void fui_path_free(fui_path_t *);

fui_path_t *fui_path_add_contours(fui_path_t *path, fui_path_t *another);

fui_path_t *fui_path_arc_move(fui_path_t *path, float angle, const float pt[2]);
fui_path_t *fui_path_arc_to(fui_path_t *path, float angle, const float pt[2]);
fui_path_t *fui_path_arc_to_xy(fui_path_t *path, float angle, float x, float y);

void fui_path_clear(fui_path_t *path);

fui_path_t *fui_path_cubic_move(fui_path_t *path, const float ct1[2], const float ct2[2], const float pt[2]);
fui_path_t *fui_path_cubic_to(fui_path_t *path, const float ct1[2], const float ct2[2], const float pt[2]);

fui_path_t *fui_path_quadratic_move(fui_path_t *path, const float ct[2], const float pt[2]);
fui_path_t *fui_path_quadratic_to(fui_path_t *path, const float ct[2], const float pt[2]);

fui_path_t *fui_path_end_contour_arc(fui_path_t *path, const float angle);
fui_path_t *fui_path_end_contour_cubic(fui_path_t *path, const float ct1[2], const float ct2[2]);
fui_path_t *fui_path_end_contour_quadratic(fui_path_t *path, const float ct[2]);

int fui_path_is_flat(const fui_path_t *path);
fui_path_t *fui_path_line_to(fui_path_t *path, const float pt[2]);
fui_path_t *fui_path_line_to_xy(fui_path_t *path, float x, float y);
fui_path_t *fui_path_move(fui_path_t *path, const float pt[2]);
fui_path_t *fui_path_move_xy(fui_path_t *path, float x, float y);
unsigned int fui_path_number_contours(const fui_path_t *path);

fui_path_t *fui_path_append_point(fui_path_t *path, const float pt[2]);
fui_path_t *fui_path_append_point_xy(fui_path_t *path, float x, float y);

fui_path_t *fui_path_append_control_point(fui_path_t *path, const float pt[2]);
fui_path_t *fui_path_append_control_point_xy(fui_path_t *path, float x, float y);

fui_path_t *fui_path_append_arc(fui_path_t *path, float angle, const float pt[2]);
fui_path_t *fui_path_append_contour_end(fui_path_t *path);

const fui_path_t* fui_path_copy(fui_path_t *path, const fui_path_t *from);

void fui_path_swap(fui_path_t *path, fui_path_t *path2);

#ifdef __cplusplus
}
#endif
