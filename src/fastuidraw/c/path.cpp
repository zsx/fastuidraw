#include "fastuidraw/c/path.h"

fui_path_t *fui_path_new()
{
    return FASTUIDRAWnew fastuidraw::Path();
}

void fui_path_free(fui_path_t * path)
{
    FASTUIDRAWdelete(path);
}
fui_path_t *fui_path_add_contours(fui_path_t *path, fui_path_t *another)
{
    return &path->add_contours(*another);
}

fui_path_t *fui_path_arc_move(fui_path_t *path, float angle, const float pt[2])
{
    return &path->arc_move(angle, fastuidraw::vec2(pt[0], pt[1]));
}

fui_path_t *fui_path_arc_to(fui_path_t *path, float angle, const float pt[2])
{
    return &path->arc_to(angle, fastuidraw::vec2(pt[0], pt[1]));
}

fui_path_t *fui_path_arc_to_xy(fui_path_t *path, float angle, float x, float y)
{
    return &path->arc_to(angle, fastuidraw::vec2(x, y));
}

void fui_path_clear(fui_path_t *path)
{
    path->clear();
}

fui_path_t *fui_path_cubic_move(fui_path_t *path, const float ct1[2], const float ct2[2], const float pt[2])
{
    return &path->cubic_move(fastuidraw::vec2(ct1[0], ct1[1]),
                             fastuidraw::vec2(ct2[0], ct2[1]),
                             fastuidraw::vec2(pt[0], pt[1]));
}

fui_path_t *fui_path_cubic_to(fui_path_t *path, const float ct1[2], const float ct2[2], const float pt[2])
{
    return &path->cubic_to(fastuidraw::vec2(ct1[0], ct1[1]),
                           fastuidraw::vec2(ct2[0], ct2[1]),
                           fastuidraw::vec2(pt[0], pt[1]));
}

fui_path_t *fui_path_quadratic_move(fui_path_t *path, const float ct[2], const float pt[2])
{
    return &path->quadratic_move(fastuidraw::vec2(ct[0], ct[1]),
                                 fastuidraw::vec2(pt[0], pt[1]));
}

fui_path_t *fui_path_quadratic_to(fui_path_t *path, const float ct[2], const float pt[2])
{
    return &path->quadratic_to(fastuidraw::vec2(ct[0], ct[1]),
                               fastuidraw::vec2(pt[0], pt[1]));
}

fui_path_t *fui_path_end_contour_arc(fui_path_t *path, const float angle)
{
    return &path->end_contour_arc(angle);
}

fui_path_t *fui_path_end_contour_cubic(fui_path_t *path, const float ct1[2], const float ct2[2])
{
    return &path->end_contour_cubic(fastuidraw::vec2(ct1[0], ct1[1]),
                            fastuidraw::vec2(ct2[0], ct2[1]));
}

fui_path_t *fui_path_end_contour_quadratic(fui_path_t *path, const float ct[2])
{
    return &path->end_contour_quadratic(fastuidraw::vec2(ct[0], ct[1]));
}

int fui_path_is_flat(const fui_path_t *path)
{
    return path->is_flat();
}

fui_path_t *fui_path_line_to(fui_path_t *path, const float pt[2])
{
    return &path->line_to(fastuidraw::vec2(pt[0], pt[1]));
}

fui_path_t *fui_path_line_to_xy(fui_path_t *path, float x, float y)
{
    return &path->line_to(fastuidraw::vec2(x, y));
}

fui_path_t *fui_path_move(fui_path_t *path, const float pt[2])
{
    return &path->move(fastuidraw::vec2(pt[0], pt[1]));
}

fui_path_t *fui_path_move_xy(fui_path_t *path, float x, float y)
{
    return &path->move(fastuidraw::vec2(x, y));
}

unsigned int fui_path_number_contours(const fui_path_t *path)
{
    return path->number_contours();
}

fui_path_t *fui_path_append_point(fui_path_t *path, const float pt[2])
{
    return &((*path) << fastuidraw::vec2(pt[0], pt[1]));
}

fui_path_t *fui_path_append_point_xy(fui_path_t *path, float x, float y)
{
    return &((*path) << fastuidraw::vec2(x, y));
}


fui_path_t *fui_path_append_control_point(fui_path_t *path, const float pt[2])
{
    return &((*path) << fastuidraw::Path::control_point(pt[0], pt[1]));
}

fui_path_t *fui_path_append_control_point_xy(fui_path_t *path, float x, float y)
{
    return &((*path) << fastuidraw::Path::control_point(x, y));
}

const fui_path_t* fui_path_copy(fui_path_t *path, const fui_path_t *from)
{
    return &((*path) = (*from));
}

void fui_path_swap(fui_path_t *path, fui_path_t *path2)
{
    path->swap(*path2);
}

fui_path_t *fui_path_append_arc(fui_path_t *path, float angle, const float pt[2])
{
    return &((*path) << fastuidraw::Path::arc(angle, fastuidraw::vec2(pt[0], pt[1])));
}

fui_path_t *fui_path_append_contour_end(fui_path_t *path)
{
    return &((*path) << fastuidraw::Path::contour_end());
}
