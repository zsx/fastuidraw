#include "fastuidraw/gl_backend/image_gl.hpp"
#include "fastuidraw/gl_backend/colorstop_atlas_gl.hpp"
#include "fastuidraw/gl_backend/glyph_atlas_gl.hpp"

fastuidraw::reference_counted_ptr<fastuidraw::gl::ImageAtlasGL> fui_imageAtlas;
fastuidraw::reference_counted_ptr<fastuidraw::gl::ColorStopAtlasGL> fui_colorStopAtlas;
fastuidraw::reference_counted_ptr<fastuidraw::gl::GlyphAtlasGL> fui_glyphAtlas;

void fui_init()
{
    fui_imageAtlas = FASTUIDRAWnew fastuidraw::gl::ImageAtlasGL(fastuidraw::gl::ImageAtlasGL::params());
    fui_colorStopAtlas = FASTUIDRAWnew fastuidraw::gl::ColorStopAtlasGL(fastuidraw::gl::ColorStopAtlasGL::params());
    fui_glyphAtlas = FASTUIDRAWnew fastuidraw::gl::GlyphAtlasGL(fastuidraw::gl::GlyphAtlasGL::params());
}

void fui_fini()
{
    FASTUIDRAWdelete(fui_imageAtlas.get());
    FASTUIDRAWdelete(fui_colorStopAtlas.get());
    FASTUIDRAWdelete(fui_glyphAtlas.get());
}
