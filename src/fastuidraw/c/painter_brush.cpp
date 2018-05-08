#include "fastuidraw/c/painter_brush.h"

fui_painter_brush_t *fui_painter_brush_new()
{
    return FASTUIDRAWnew fastuidraw::PainterBrush();
}

fui_painter_brush_t *fui_painter_brush_new_with_pen(const float color[4])
{
    return FASTUIDRAWnew fastuidraw::PainterBrush(fastuidraw::vec4(color[0], color[1], color[2], color[3]));
}

void fui_painter_brush_free(fui_painter_brush_t *brush)
{
    FASTUIDRAWdelete(brush);
}

fui_painter_brush_t *fui_painter_brush_set_image(fui_painter_brush_t *brush,
                                                 const fui_image_t *image,
                                                 int filter)
{
    reference_counted_ptr<const fastuidraw::Image> rc(image);
    if (filter < 0) {
        return &brush->image(rc);
    } else {
        return &brush->image(rc, static_cast<fastuidraw::PainterBrush::image_filter>(filter));
    }
}


fui_painter_brush_t *fui_painter_brush_set_sub_image(fui_painter_brush_t *brush,
                                                    const fui_image_t *image,
                                                    uint32_t x, uint32_t y,
                                                    uint32_t w, uint32_t h,
                                                    int filter)
{
    reference_counted_ptr<const fastuidraw::Image> rc(image);
    fastuidraw::uvec2 xy(x, y);
    fastuidraw::uvec2 wh(w, h);
    if (filter < 0) {
        return &brush->sub_image(rc, xy, wh);
    } else {
        return &brush->sub_image(rc, xy, wh, static_cast<fastuidraw::PainterBrush::image_filter>(filter));
    }
}

fui_painter_brush_t *fui_painter_brush_set_pen(fui_painter_brush_t *brush, const float color[4])
{
    return &brush->pen(fastuidraw::vec4(color[0], color[1], color[2], color[3]));
}
fui_painter_brush_t *fui_painter_brush_set_pen_rgba(fui_painter_brush_t *brush, float r, float g, float b, float a)
{
    return &brush->pen(r, g, b, a);
}

fui_painter_brush_t *fui_painter_brush_set_transformation_translate_xy(fui_painter_brush_t *brush, float x, float y)
{
    return &brush->transformation_translate(fastuidraw::vec2(x, y));
}

fui_painter_brush_t *fui_painter_brush_set_linear_gradient(fui_painter_brush_t *brush,
                                                           fui_colorstop_sequence_on_atlas_t *seq,
                                                           float x1, float y1,
                                                           float x2, float y2,
                                                           int repeat)
{
    return &brush->linear_gradient(fastuidraw::reference_counted_ptr<const fastuidraw::ColorStopSequenceOnAtlas>(seq),
                                   fastuidraw::vec2(x1, y1),
                                   fastuidraw::vec2(x2, y2),
                                   repeat);
}

fui_painter_brush_t *fui_painter_brush_set_radial_gradient(fui_painter_brush_t *brush,
                                                           fui_colorstop_sequence_on_atlas_t *seq,
                                                           float x1, float y1, float r1,
                                                           float x2, float y2, float r2,
                                                           int repeat)
{
    return &brush->radial_gradient(fastuidraw::reference_counted_ptr<const fastuidraw::ColorStopSequenceOnAtlas>(seq),
                                   fastuidraw::vec2(x1, y1), r1,
                                   fastuidraw::vec2(x2, y2), r2,
                                   repeat);
}

fui_painter_brush_t *fui_painter_brush_set_repeat_window(fui_painter_brush_t *brush,
                                                         float x, float y,
                                                         float w, float h)
{
    return &brush->repeat_window(fastuidraw::vec2(x, y), fastuidraw::vec2(w, h));
}

fui_painter_brush_t *fui_painter_brush_set_no_image(fui_painter_brush_t *brush)
{
    return &brush->no_image();
}
fui_painter_brush_t *fui_painter_brush_set_no_gradient(fui_painter_brush_t *brush)
{
    return &brush->no_gradient();
}
fui_painter_brush_t *fui_painter_brush_set_no_repeat_window(fui_painter_brush_t *brush)
{
    return &brush->no_repeat_window();
}
fui_painter_brush_t *fui_painter_brush_set_no_transformation(fui_painter_brush_t *brush)
{
    return &brush->no_transformation();
}
fui_painter_brush_t *fui_painter_brush_set_no_transformation_matrix(fui_painter_brush_t *brush)
{
    return &brush->no_transformation_matrix();
}
fui_painter_brush_t *fui_painter_brush_set_no_transformation_translation(fui_painter_brush_t *brush)
{
    return &brush->no_transformation_translation();
}

void fui_painter_brush_reset(fui_painter_brush_t *brush)
{
    brush->reset();
}
