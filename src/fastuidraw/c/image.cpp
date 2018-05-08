#include <cstring>
#include "fastuidraw/c/image.h"
#include <vector>

fui_image_t *fui_image_new_from_RGBA8888(fui_image_atlas_t* atlas, int w, int h, int flip_y, const char *data, unsigned int pslack)
{
    reference_counted_ptr<ImageAtlas> rc(atlas);
    std::vector<fastuidraw::u8vec4> vdata(w * h);
    for(auto y = 0; y < h; ++ y) {
        int s_y = y;
        if (flip_y) s_y = h - y;
        for(auto x = 0; x < w; ++ x) {
            auto s_i = (s_y * w + x) * 4;
            auto d_i = y * w + x;
            vdata[d_i][0] = data[s_i + 0];
            vdata[d_i][1] = data[s_i + 1];
            vdata[d_i][2] = data[s_i + 2];
            vdata[d_i][3] = data[s_i + 3];
        }
    }
    auto r = Image::create(rc, w, h, c_array<const u8vec4>(&vdata[0], vdata.size()), pslack);
    auto p = r.get();
    p->add_reference(p);
    return p;
}

void fui_image_free(fui_image_t *image)
{
    image->remove_reference(image);
}

void fui_image_dimensions(const fui_image_t *image, int *w, int *h)
{
    auto d = image->dimensions();
    if (w) *w = d.x();
    if (h) *w = d.y();
}
