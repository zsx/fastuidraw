LIB_OBJS=colorstop_atlas_gl.o colorstop_atlas_gl_params.o configuration_gl.o glyph_atlas_gl.o glyph_atlas_gl_params.o image_atlas_gl.o image_atlas_gl_params.o image.o painter_backend_gl.o painter_brush.o painter.o painter_stroke_params.o painter_dashed_stroke_params.o path.o gl_binding.o image_loader.o
APP_OBJS=test.o
test-debug: ${LIB_OBJS} ${APP_OBJS}
	g++ -o $@ ${LIB_OBJS} ${APP_OBJS} -L ../../.. -lFastUIDrawGL_debug -lFastUIDraw_debug -lNGL_debug `pkg-config sdl2 --libs` -lGL `pkg-config libturbojpeg --libs`
test: ${LIB_OBJS} ${APP_OBJS}
	g++ -o $@ ${LIB_OBJS} ${APP_OBJS} -L ../../.. -lFastUIDrawGL_release -lFastUIDraw_release -lNGL_release `pkg-config sdl2 --libs` -lGL

%.o: %.cpp
	g++ -c $< -I ../../../inc -ggdb -g3 -Wall -Wextra

test.o: test.c
	gcc -c $< -I ../../../inc `pkg-config sdl2 --cflags` -ggdb -g3

image_loader.o: image_loader.c
	gcc -c $< -I ../../../inc -ggdb -g3

clean:
	rm -f test *.o
