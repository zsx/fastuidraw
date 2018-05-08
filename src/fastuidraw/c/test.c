#include "fastuidraw/c/colorstop_atlas_gl.h"
#include "fastuidraw/c/glyph_atlas_gl.h"
#include "fastuidraw/c/image_atlas_gl.h"
#include "fastuidraw/c/painter_backend_gl.h"
#include "fastuidraw/c/painter.h"
#include "fastuidraw/c/painter_brush.h"
#include "fastuidraw/c/gl_binding.h"
#include "fastuidraw/c/path.h"
#include "fastuidraw/c/painter_stroke_params.h"
#include "fastuidraw/c/image.h"
#include "fastuidraw/c/image_loader.h"
#include <SDL.h>
#include <SDL_opengl.h>

typedef struct {
    fui_gl_image_atlas_gl_t *image_atlas;
    fui_gl_colorstop_atlas_gl_t *colorstop_atlas;
    fui_gl_glyph_atlas_gl_t *glyph_atlas;
    fui_painter_t *painter;
    fui_image_t *image;
} fui_context_t;

static fui_context_t *fui_context_create()
{
    fui_context_t *ctx = malloc(sizeof(fui_context_t));

    fui_gl_binding_get_proc_function(SDL_GL_GetProcAddress, 1);

    fui_gl_image_atlas_gl_params_t *image_params = fui_gl_image_atlas_gl_params_new();
    ctx->image_atlas = fui_gl_image_atlas_gl_new(image_params);
    fui_gl_image_atlas_gl_params_free(image_params);

    fui_gl_colorstop_atlas_gl_params_t *colorstop_params = fui_gl_colorstop_atlas_gl_params_new();
    ctx->colorstop_atlas = fui_gl_colorstop_atlas_gl_new(colorstop_params);
    fui_gl_colorstop_atlas_gl_params_free(colorstop_params);

    fui_gl_glyph_atlas_gl_params_t *glyph_params = fui_gl_glyph_atlas_gl_params_new();
    ctx->glyph_atlas = fui_gl_glyph_atlas_gl_new(glyph_params);
    fui_gl_glyph_atlas_gl_params_free(glyph_params);
    
    fui_gl_configuration_gl_t *conf = fui_gl_configuration_gl_new();
    fui_gl_configuration_gl_set_image_atlas(conf, ctx->image_atlas);

    fui_gl_configuration_gl_set_colorstop_atlas(conf, ctx->colorstop_atlas);

    fui_gl_configuration_gl_set_glyph_atlas(conf, ctx->glyph_atlas);

    //fui_gl_configuration_gl_set_attributes_per_buffer(conf, 512);
    fui_gl_painter_backend_gl_t *backend = fui_gl_painter_backend_gl_new(conf);
    fui_gl_configuration_gl_free(conf);

    ctx->painter = fui_painter_new_gl(backend);
    fui_gl_painter_backend_gl_free(backend);

    FILE *fp = fopen("tiger.jpg", "rb");
    if (fp == NULL) {
        printf("Can't open tiger.jpg\n");
        exit(1);
    }
    fseek(fp, 0L, SEEK_END);
    size_t jpeg_len = ftell(fp);
    rewind(fp);

    char *jpeg_data = malloc(jpeg_len);
    fread(jpeg_data, 1, jpeg_len, fp);
    fclose(fp);

    int w, h;
    unsigned char* img_data = load_jpeg(jpeg_data, jpeg_len, &w, &h);
    free(jpeg_data);

    fp = fopen("tiger.bmp", "wb");
    fwrite(img_data, 4, w * h, fp);
    fclose(fp);

    ctx->image = fui_image_new_from_RGBA8888((fui_image_atlas_t*)ctx->image_atlas, w, h, 1, img_data, 0);
    unload_jpeg(img_data);

    return ctx;
}

static void fui_context_free(fui_context_t *ctx)
{
    fui_gl_glyph_atlas_gl_free(ctx->glyph_atlas);
    fui_gl_colorstop_atlas_gl_free(ctx->colorstop_atlas);
    fui_gl_image_atlas_gl_free(ctx->image_atlas);
    fui_painter_free(ctx->painter);
    fui_image_free(ctx->image);
}

static void draw(fui_context_t *ctx, int w, int h)
{
    fui_path_t *path = fui_path_new();

    //fui_painter_set_target_resolution(painter, 1600, 1200);
    //fui_painter_set_target_resolution(ctx->painter, w, h);
    fui_gl_surface_gl_properties_t *p = fui_gl_surface_gl_properties_new();
    fui_gl_surface_gl_properties_set_dimensions_xy(p, w, h);
    fui_gl_surface_gl_t *s = fui_gl_surface_gl_new(p);
    fui_gl_surface_gl_properties_free(p);

    glClearColor(0,0,0,0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    fui_painter_begin(ctx->painter, s, 1);

    fui_painter_save(ctx->painter);
    float identity[][3] = {{2.0/w, 0, 0}, {0, 2.0/h, 0}, {-1, -1, 1}};
    fui_painter_set_transformation_3x3(ctx->painter, identity);

    fui_path_move_xy(path, 30, 40);

    fui_path_line_to_xy(path, 100, 20);
    fui_path_line_to_xy(path, 400, 200);
    fui_path_line_to_xy(path, 400, 400);
    fui_path_line_to_xy(path, 200, 500);
    fui_path_line_to_xy(path, 250, 300);
    //fui_path_line_to_xy(path, 100, 200);

    //fui_path_quadratic_to(path, ct, pt);

    //fui_path_end_contour_arc(path, 45.0 * M_PI / 180.0);
    fui_path_append_contour_end(path);

    fui_path_move_xy(path, 600, 400);
    fui_path_line_to_xy(path, 500, 300);
    fui_path_append_contour_end(path);

    fui_painter_brush_t *brush = fui_painter_brush_new();

    fui_colorstop_sequence_t *cs_seq = fui_colorstop_sequence_new();
    fui_colorstop_sequence_add_components(cs_seq, 255, 0, 0, 255, 0.0f);
    fui_colorstop_sequence_add_components(cs_seq, 0, 255, 0, 255, 0.33f);
    fui_colorstop_sequence_add_components(cs_seq, 0, 0, 255, 255, 0.66f);
    fui_colorstop_sequence_add_components(cs_seq, 255, 255, 255, 255, 1.0f);
    fui_colorstop_sequence_on_atlas_t *cs_on_atlas = fui_colorstop_sequence_on_atlas_new_gl(cs_seq, ctx->colorstop_atlas, 8);
    fui_colorstop_sequence_free(cs_seq);

    fui_painter_brush_set_linear_gradient(brush, cs_on_atlas, 100, 20, 400, 400, 0);
    fui_colorstop_sequence_on_atlas_free(cs_on_atlas);
    //fui_painter_brush_set_pen_rgba(brush, 1, 0, 0, 1);

    fui_painter_fill_path(ctx->painter, brush, path, 0, 0);

    fui_path_t *path2 = fui_path_new();
    fui_path_move_xy(path2, 50, 300);

    fui_path_line_to_xy(path2, 50, 825);
    fui_path_line_to_xy(path2, 475, 825);
    fui_path_line_to_xy(path2, 475, 300);
    fui_path_append_contour_end(path2);

    fui_painter_brush_set_no_gradient(brush);
    fui_painter_brush_set_pen_rgba(brush, 1, 1, 1, 1);
    //fui_painter_brush_set_sub_image(brush, ctx->image, 0, 0, 225, 225, -1);
    fui_painter_brush_set_image(brush, ctx->image, -1);
    //fui_painter_brush_set_transformation_translate_xy(brush, -50, -300);
    fui_painter_fill_path(ctx->painter, brush, path2, 0, 1);
    fui_path_free(path2);

    float xy[] = {0, 0};
    float wh[] = {300, 600};
    fui_painter_brush_set_pen_rgba(brush, 0, 1, 1, 1);
    fui_painter_draw_rect(ctx->painter, brush, xy, wh);

    fui_painter_brush_set_no_gradient(brush);
    fui_painter_brush_set_pen_rgba(brush, 0, 1, 0, 1);

    fui_painter_stroke_params_t *stroke_params = fui_painter_stroke_params_new();
    fui_painter_stroke_params_set_width(stroke_params, 10);
    //fui_painter_stroke_path(ctx->painter, path, brush, stroke_params, 1, 1, 1, 1);
    
    fui_painter_dashed_stroke_params_t *dashed_params = fui_painter_dashed_stroke_params_new();
    fui_painter_dashed_stroke_params_set_width(dashed_params, 8);
    float dashed_patterns[][2] = {{50, 30}, {3, 10}, { 8, 20}};
    fui_painter_dashed_stroke_params_set_dash_pattern(dashed_params, dashed_patterns, sizeof(dashed_patterns)/sizeof(dashed_patterns[0]));
    
    fui_painter_stroke_dashed_path(ctx->painter, path, brush, dashed_params, 1, 1, 1, 1);
    fui_painter_dashed_stroke_params_free(dashed_params);

#if 0
    fui_path_clear(path);
    fui_path_move_xy(path, 100, 300);
    fui_path_arc_to_xy(path, M_PI, 100, 400);
    fui_path_end_contour_arc(path, M_PI);

    fui_painter_brush_set_pen_rgba(brush, 1, 1, 0, 1);
    fui_painter_stroke_path(ctx->painter, path, brush, stroke_params, 1, 1, 1, 1);
#endif

    xy[0] = 440; xy[1] = 50;
    wh[0] = 300; wh[1] = 200;
    fui_painter_brush_set_pen_rgba(brush, 0, 1, 1, 1);
    fui_painter_draw_rect(ctx->painter, brush, xy, wh);

    fui_painter_translate_xy(ctx->painter, 0, 220);
    //fui_painter_rotate(ctx->painter, 5.0 * M_PI / 180.0);
    fui_painter_draw_rect(ctx->painter, brush, xy, wh);

    //fui_painter_translate_xy(ctx->painter, -440, -270);
    //fui_painter_rotate(ctx->painter, 5.0 * M_PI / 180.0);
    //fui_painter_draw_rect(ctx->painter, brush, xy, wh);

    fui_path_free(path);
    fui_painter_brush_free(brush);
    fui_painter_stroke_params_free(stroke_params);

    fui_painter_restore(ctx->painter);
    fui_painter_end(ctx->painter);
    fui_gl_surface_gl_blit_surface(s);
    fui_gl_surface_gl_free(s);
}

int main ()
{
    const int w = 800, h = 600;
    SDL_Init(SDL_INIT_VIDEO | SDL_INIT_EVENTS);
//    SDL_GL_SetAttribute(SDL_GL_RED_SIZE, 5);
//    SDL_GL_SetAttribute(SDL_GL_GREEN_SIZE, 5);
//    SDL_GL_SetAttribute(SDL_GL_BLUE_SIZE, 5);
//    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 16);
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 4);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 5);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, SDL_GL_CONTEXT_DEBUG_FLAG);

    SDL_Window *win = SDL_CreateWindow("FastUIDraw", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, w, h, SDL_WINDOW_OPENGL);
    SDL_GLContext gl_ctx = SDL_GL_CreateContext(win);
    //glViewport(0, 0, w, h);
    glClearDepth(0.0);
    glClearColor(0,0,0,0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    SDL_GL_SwapWindow(win);

    printf("OpenGL version: %s\n", glGetString(GL_VERSION));

    //SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_INFORMATION, "VERSION:", "4.5", win);

    fui_context_t *ctx = fui_context_create();

    int running = 5;
    while(running) {
        SDL_Event event;
        while (running && SDL_PollEvent(&event)) {
            /* handle your event here */
            switch(event.type) {
                case SDL_QUIT:
                    running = 0;
                    break;
                default:
                    ;
            }
        }
        draw(ctx, w, h);
        SDL_GL_SwapWindow(win);
        //-- running;
    }

    fui_context_free(ctx);

    SDL_GL_DeleteContext(gl_ctx);
    SDL_DestroyWindow(win);
    SDL_Quit();
    return 0;
}
