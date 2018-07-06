#include <cstring>

#include "fastuidraw/c/painter.h"
#include "fastuidraw/util/util.hpp"
#include "fastuidraw/util/fastuidraw_memory.hpp"
#include "fastuidraw/gl_backend/image_gl.hpp"
#include "fastuidraw/gl_backend/colorstop_atlas_gl.hpp"
#include "fastuidraw/gl_backend/glyph_atlas_gl.hpp"
#include "fastuidraw/gl_backend/painter_backend_gl.hpp"


fui_painter_t *fui_painter_new_gl(fui_gl_painter_backend_gl_t *backend)
{
    fastuidraw::reference_counted_ptr<fastuidraw::gl::PainterBackendGL> painterBackend(backend);
    auto p = FASTUIDRAWnew fastuidraw::Painter(painterBackend);
    //p->add_reference(p);

    return p;
}

void fui_painter_free(fui_painter_t *painter)
{
    FASTUIDRAWdelete(painter);
    //painter->remove_reference(painter);
}

int fui_painter_current_z(const fui_painter_t *painter)
{
    return painter->current_z();
}

void fui_painter_increment_z(fui_painter_t *painter, int amount)
{
    painter->increment_z(amount);
}

void fui_painter_curveFlatness_set(fui_painter_t *painter, float thresh)
{
    painter->curveFlatness(thresh);
}

float fui_painter_curveFlatness_get(fui_painter_t *painter)
{
    return painter->curveFlatness();
}

/*
void fui_painter_set_target_resolution(fui_painter_t * painter, int w, int h)
{
    painter->target_resolution(w, h);
}
*/

void fui_painter_begin(fui_painter_t *painter, fui_gl_surface_gl_t *surface, int reset_z)
{
    const reference_counted_ptr<PainterBackend::Surface> rc(surface);
    painter->begin(rc, reset_z);
}

void fui_painter_end(fui_painter_t *painter)
{
    painter->end();
}

void fui_painter_restore(fui_painter_t *painter)
{
    painter->restore();
}

void fui_painter_rotate(fui_painter_t *painter, float angle)
{
    painter->rotate(angle);
}

void fui_painter_save(fui_painter_t *painter)
{
    painter->save();
}

void fui_painter_scale(fui_painter_t *painter, float s)
{
    painter->scale(s);
}

void fui_painter_shear(fui_painter_t *painter, float sx, float sy)
{
    painter->shear(sx, sy);
}

void fui_painter_set_transformation_3x3(fui_painter_t *painter, const float m[3][3])
{
    fastuidraw::float3x3 m3x3;
    std::memcpy(m3x3.c_ptr(), m, sizeof(float) * 9);
    painter->transformation(m3x3);
}

void fui_painter_get_transformation_3x3(fui_painter_t *painter, float m[3][3])
{
    auto mtx = painter->transformation();
    c_array<generic_data> array(reinterpret_cast<generic_data*>(m), 9);
    mtx.pack_data(sizeof(float), array);
}

void fui_painter_translate(fui_painter_t *painter, const float p[2])
{
    painter->translate(fastuidraw::vec2(p[0], p[1]));
}

void fui_painter_translate_xy(fui_painter_t *painter, float x, float y)
{
    painter->translate(fastuidraw::vec2(x, y));
}

void fui_painter_clipInRect(fui_painter_t *painter, const float xy[2], const float wh[2])
{
    painter->clipInRect(fastuidraw::vec2(xy[0], xy[1]), fastuidraw::vec2(wh[0], wh[1]));
}

void fui_painter_clipInPath(fui_painter_t *painter, const fui_path_t *path, int fill_rule)
{
    painter->clipInPath(*path, static_cast<fastuidraw::PainterEnums::fill_rule_t>(fill_rule));
}

void fui_painter_clipOutPath(fui_painter_t *painter, const fui_path_t *path, int fill_rule)
{
    painter->clipOutPath(*path, static_cast<fastuidraw::PainterEnums::fill_rule_t>(fill_rule));
}

void fui_painter_draw_rect(fui_painter_t *painter,
                           fui_painter_brush_t *brush,
                           const float p[2],
                           const float wh[2])
{
    painter->draw_rect(fastuidraw::PainterData(brush), fastuidraw::vec2(p[0], p[1]), fastuidraw::vec2(wh[0], wh[1]));
}

void fui_painter_stroke_path(fui_painter_t *painter,
                             const fui_path_t *path,
                             const fui_painter_brush_t *brush,
                             const fui_painter_stroke_params_t *stroke_params,
                             int close_contours,
                             int cap,
                             int join_style,
                             int anti_aliasing)
{
    painter->stroke_path(fastuidraw::PainterData(brush, stroke_params),
                         *path,
                         close_contours,
                         static_cast<fastuidraw::PainterEnums::cap_style>(cap),
                         static_cast<fastuidraw::PainterEnums::join_style>(join_style),
                         anti_aliasing);

}

void fui_painter_fill_path(fui_painter_t *painter,
                           const fui_painter_brush_t *brush,
                           const fui_path_t *path,
                           int fill_rule,
                           int with_anti_aliasing)
{
    painter->fill_path(fastuidraw::PainterData(brush),
                       *path,
                       static_cast<fastuidraw::PainterEnums::fill_rule_t>(fill_rule),
                       with_anti_aliasing);
}

void fui_painter_stroke_dashed_path(fui_painter_t *painter,
                                    const fui_path_t *path,
                                    const fui_painter_brush_t *brush,
                                    const fui_painter_dashed_stroke_params_t *stroke_params,
                                    int close_contours,
                                    int cap,
                                    int join_style,
                                    int anti_aliasing)
{
    painter->stroke_dashed_path(fastuidraw::PainterData(brush, stroke_params),
                                *path,
                                close_contours,
                                static_cast<fastuidraw::PainterEnums::cap_style>(cap),
                                static_cast<fastuidraw::PainterEnums::join_style>(join_style),
                                anti_aliasing);
}
