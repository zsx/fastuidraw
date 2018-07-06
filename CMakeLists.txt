# vim: ts=4 shiftwidth=4 filetype=cmake expandtab

#
# Sample command lines:
#
#     cd make
#     mkdir build
#     cd build
#     CMake .. -G "Visual Studio 14 Win64" -DR3_OS_ID=0.3.40
#          -DR3_EXTERNAL_FFI=yes -DR3_CPP=no
#

cmake_minimum_required (VERSION 2.8)

project (FastUIDraw C CXX)

set(GL_INC_DIR "" CACHE PATH "directory for OpenGL header files")
set(EGL_INC_DIR "" CACHE PATH "directory for EGL header files")

set (TOP_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
set (TOP_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src")
set (TOP_INC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/inc")
set (TOP_DEMOS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/demos")
set (SDL2_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../SDL2-2.0.5")
set (SDL2_LIB "${SDL2_DIR}/lib/x64/SDL2.lib;${SDL2_DIR}/lib/x64/SDL2main.lib")
set (SDL2_IMAGE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../SDL2_image-2.0.1")
set (SDL2_IMAGE_LIB "${SDL2_IMAGE_DIR}/lib/x64/SDL2_image.lib")
set (BOOST_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../boost_1_63_0")
set (BOOST_LIB_DIR "${BOOST_DIR}/stage/lib")
set (FT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../freetype-2.7.1" CACHE PATH "root directory of freetype")
set (FT_INC_DIR "${FT_DIR}/include" CACHE PATH "directory for freetype header files")
set (FT_LIB_DIR "${FT_DIR}/lib" CACHE PATH "Freetype library file")
set (FT_LIB "freetype.lib" CACHE STRING "Freetype library file")
set (GL_RAW_HEADER_FILES "GL/glcorearb.h" CACHE LIST "OpenGL header files")

set (DIRENT_INC_DIR "../dirent.git/include" CACHE PATH "directory for dirent.h")
set (FLEX "flex" CACHE FILE "The executable file for flex")

set (THIRD_PARTY_DIR ${TOP_SRC_DIR}/3rd_party)
set (GLU_TESS_DIR ${THIRD_PARTY_DIR}/glu-tess)
set (FASTUIDRAW_SRC_DIR ${TOP_SRC_DIR}/fastuidraw)
set (FASTUIDRAW_INC_DIR ${TOP_INC_DIR}/fastuidraw)
set (PAINTER_SRC_DIR ${FASTUIDRAW_SRC_DIR}/painter)
set (GLSL_SRC_DIR ${FASTUIDRAW_SRC_DIR}/glsl)
set (GL_BACKEND_SRC_DIR ${FASTUIDRAW_SRC_DIR}/gl_backend)
set (GL_BACKEND_INC_DIR ${FASTUIDRAW_INC_DIR}/gl_backend)
set (PRIVATE_SRC_DIR ${FASTUIDRAW_SRC_DIR}/private)
set (TEXT_SRC_DIR ${FASTUIDRAW_SRC_DIR}/text)
set (UTIL_SRC_DIR ${FASTUIDRAW_SRC_DIR}/util)
set (FASTUIDRAW_RES_DIR ${CMAKE_BINARY_DIR}/string_resources_cpp)
set (NGL_GENERATOR_DIR ${CMAKE_BINARY_DIR}/ngl_generator)

#link_directories(${BOOST_LIB_DIR} ${FT_LIB_DIR})
link_directories(${FT_LIB_DIR})

macro(add_project name type sources)
    message("sources for ${name}: ${sources}")
    add_library(${name} ${type} ${sources})
    target_include_directories(${name} PUBLIC
        ${TOP_INC_DIR}
        #${BOOST_DIR}
        ${GL_INC_DIR}
        ${FREETYPE_INC_DIR}
        ${DIRENT_INC_DIR}
    )
    list(APPEND ALL_PROJECTS ${name})
endmacro ()

macro(add_object_project name sources)
    add_project(${name} OBJECT "${sources}")
endmacro ()

macro(add_shader_source shader)
    get_filename_component(dirname ${shader} DIRECTORY)
    get_filename_component(filename ${shader} NAME)
    make_directory($dirname)
    add_custom_command(
        OUTPUT ${FASTUIDRAW_RES_DIR}/${shader}.cpp
        COMMAND $<TARGET_FILE:${STRING_RESOURCE_CC}> ${TOP_DIR}/${shader} ${filename} ${FASTUIDRAW_RES_DIR}/${dirname}
        DEPENDS ${STRING_RESOURCE_CC}
    )
endmacro()

macro(add_demo_project name sources shader_sources)
    set(demo_generated_sources "")
    if (NOT shader_sources)
        message("adding shader_sources: ${shader_sources}")
        set (demo_shaders ${shader_sources})
        set (demo_generated_sources "")
        foreach(shader IN LISTS demo_shaders)
            message(STATUS "demo shader: ${shader}")
            add_shader_source(${shader})
            list(APPEND demo_generated_sources ${FASTUIDRAW_RES_DIR}/${shader}.cpp)
        endforeach(shader)
    endif()

    message("demo_generated_sources: ${demo_generated_sources}")
    add_executable(${name} $<TARGET_OBJECTS:demo-common> "${sources};${demo_generated_sources}")
    target_include_directories(${name} PRIVATE
        "${SDL2_DIR}/include"
        "${SDL2_IMAGE_DIR}/include"
        "${FT_INC_DIR}"
        ${TOP_DEMOS_DIR}/common
        )
    target_compile_definitions(${name} PRIVATE
        #"FASTUIDRAW_GL_API=__declspec(dllimport)"
        )
    target_link_libraries(${name}
        FastUIDrawGL
        ${SDL2_LIB}
        ${SDL2_IMAGE_LIB}
        ${FT_LIB}
        )
    list(APPEND ALL_PROJECTS ${name})
endmacro ()

set(STRING_RESOURCE_CC fastuidraw-create-resource-cpp-file)
add_executable(
    ${STRING_RESOURCE_CC}
    shell_scripts/${STRING_RESOURCE_CC}.cpp
)

set(NGL_FILTER ngl-filter)
add_executable(${NGL_FILTER}
    ${FASTUIDRAW_SRC_DIR}/ngl_generator/filter.cpp
)

set(NGL_EXTRACTOR ngl-extractor)
add_custom_command(OUTPUT ${NGL_GENERATOR_DIR}/gl_flex.cpp
    COMMAND ${FLEX} -o ${NGL_GENERATOR_DIR}/gl_flex.cpp ${FASTUIDRAW_SRC_DIR}/ngl_generator/gl_flex.fl.cpp
    DEPENDS
    ${FASTUIDRAW_SRC_DIR}/ngl_generator/gl_flex.fl.cpp
)
add_executable(${NGL_EXTRACTOR}
    ${FASTUIDRAW_SRC_DIR}/ngl_generator/HeaderCreator.cpp
    ${NGL_GENERATOR_DIR}/gl_flex.cpp
)
target_include_directories(${NGL_EXTRACTOR} PUBLIC ${FASTUIDRAW_SRC_DIR}/ngl_generator)
    
set (GLU_TESS_SOURCES
    ${GLU_TESS_DIR}/dict.cpp
    ${GLU_TESS_DIR}/geom.cpp
    ${GLU_TESS_DIR}/memalloc.cpp
    ${GLU_TESS_DIR}/mesh.cpp
    ${GLU_TESS_DIR}/priorityq.cpp
    ${GLU_TESS_DIR}/render.cpp
    ${GLU_TESS_DIR}/sweep.cpp
    ${GLU_TESS_DIR}/tess.cpp
    ${GLU_TESS_DIR}/tessmono.cpp
)

set (NGL_GL_CPP ${GL_BACKEND_SRC_DIR}/ngl_gl.cpp)
set (NGL_GL_HPP ${GL_BACKEND_INC_DIR}/ngl_gl.hpp)
set (GL_HEADER_FILES "")
foreach (header IN LISTS GL_RAW_HEADER_FILES)
    LIST(APPEND GL_HEADER_FILES ${GL_INC_DIR}/${header})
endforeach()
file(TO_NATIVE_PATH $<TARGET_FILE:${NGL_EXTRACTOR}> NGL_EXTRACTOR_EXE)
message(STATUS "NGL_EXTRACTOR_EXE: ${NGL_EXTRACTOR_EXE}")
add_custom_command(
    OUTPUT ${NGL_GL_CPP} ${NGL_GL_HPP}
    COMMAND $<TARGET_FILE:${NGL_FILTER}> ${GL_HEADER_FILES} > ${CMAKE_BINARY_DIR}/filtered_header.hpp
    COMMAND ${NGL_EXTRACTOR_EXE} macro_prefix=FASTUIDRAWgl namespace="fastuidraw::gl_binding" path=${GL_INC_DIR} output_cpp=${NGL_GL_CPP} output_hpp=${NGL_GL_HPP} ${GL_RAW_HEADER_FILES} < ${CMAKE_BINARY_DIR}/filtered_header.hpp
    DEPENDS
    ${NGL_FILTER}
    ${NGL_EXTRACTOR}
    ${GL_RAW_HEADER_FILES}
    COMMAND_EXPAND_LISTS
)
set (NGL_SOURCES
    ${GL_BACKEND_SRC_DIR}/gl_binding.cpp
    ${NGL_GL_CPP}
)

add_project(NGL STATIC "${NGL_SOURCES}")
if (MSVC)
    target_compile_definitions(NGL PRIVATE
        #"GLAPI=__declspec(dllexport)"
        #"FASTUIDRAW_API=__declspec(dllimport)"
        #"FASTUIDRAW_NGL_API=__declspec(dllexport)"
        "FASTUIDRAW_API"
        "FASTUIDRAW_NGL_API"
        )
endif ()

set (GLSL_SOURCES
    ${GLSL_SRC_DIR}/shader_source.cpp
    ${GLSL_SRC_DIR}/shader_code.cpp
    ${GLSL_SRC_DIR}/painter_item_shader_glsl.cpp
    ${GLSL_SRC_DIR}/painter_blend_shader_glsl.cpp
    ${GLSL_SRC_DIR}/painter_backend_glsl.cpp
    ${GLSL_SRC_DIR}/private/backend_shaders.cpp
    ${GLSL_SRC_DIR}/private/uber_shader_builder.cpp
)

set (SHADERS
    src/fastuidraw/glsl/shaders/fastuidraw_align.vert.glsl.resource_string
    src/fastuidraw/glsl/shaders/fastuidraw_atlas_image_fetch.glsl.resource_string
    src/fastuidraw/glsl/shaders/fastuidraw_compute_local_distance_from_pixel_distance.glsl.resource_string
    src/fastuidraw/glsl/shaders/fastuidraw_curvepair_glyph_derivative.frag.glsl.resource_string
    src/fastuidraw/glsl/shaders/fastuidraw_anisotropic.frag.glsl.resource_string
    src/fastuidraw/glsl/shaders/fastuidraw_circular_interpolate.glsl.resource_string
    src/fastuidraw/glsl/shaders/fastuidraw_curvepair_glyph.frag.glsl.resource_string
    src/fastuidraw/glsl/shaders/fastuidraw_unpack_unit_vector.glsl.resource_string
)

set (PAINTER_SOURCES
    ${PAINTER_SRC_DIR}/fill_rule.cpp
	${PAINTER_SRC_DIR}/painter_attribute_data.cpp
	${PAINTER_SRC_DIR}/painter_attribute_data_filler_glyphs.cpp
	${PAINTER_SRC_DIR}/painter_brush.cpp
    ${PAINTER_SRC_DIR}/painter_stroke_params.cpp
	${PAINTER_SRC_DIR}/painter_dashed_stroke_params.cpp
	${PAINTER_SRC_DIR}/painter.cpp
    ${PAINTER_SRC_DIR}/painter_enums.cpp
	${PAINTER_SRC_DIR}/painter_shader_data.cpp
	${PAINTER_SRC_DIR}/painter_clip_equations.cpp
	${PAINTER_SRC_DIR}/painter_item_matrix.cpp
    ${PAINTER_SRC_DIR}/painter_header.cpp
	${PAINTER_SRC_DIR}/painter_shader.cpp
    ${PAINTER_SRC_DIR}/painter_shader_set.cpp
	${PAINTER_SRC_DIR}/painter_dashed_stroke_shader_set.cpp
    ${PAINTER_SRC_DIR}/painter_stroke_shader.cpp
	${PAINTER_SRC_DIR}/painter_glyph_shader.cpp
    ${PAINTER_SRC_DIR}/painter_blend_shader_set.cpp
	${PAINTER_SRC_DIR}/painter_fill_shader.cpp
	${PAINTER_SRC_DIR}/stroked_caps_joins.cpp
	${PAINTER_SRC_DIR}/stroked_point.cpp
	${PAINTER_SRC_DIR}/stroked_path.cpp
    ${PAINTER_SRC_DIR}/filled_path.cpp
	${PAINTER_SRC_DIR}/arc_stroked_point.cpp
    ${PAINTER_SRC_DIR}/packing/painter_backend.cpp
    ${PAINTER_SRC_DIR}/packing/painter_draw.cpp
    ${PAINTER_SRC_DIR}/packing/painter_packer.cpp
)

set (PAINTER_SHADERS
    #painter shaders
    src/fastuidraw/glsl/shaders/painter/fastuidraw_painter_main.vert.glsl.resource_string 
    src/fastuidraw/glsl/shaders/painter/fastuidraw_painter_types.glsl.resource_string 
    src/fastuidraw/glsl/shaders/painter/fastuidraw_painter_forward_declares.vert.glsl.resource_string 
    src/fastuidraw/glsl/shaders/painter/fastuidraw_painter_uniforms.glsl.resource_string 
    src/fastuidraw/glsl/shaders/painter/fastuidraw_painter_auxiliary_image_buffer.glsl.resource_string 
    src/fastuidraw/glsl/shaders/painter/fastuidraw_painter_forward_declares.frag.glsl.resource_string 
    src/fastuidraw/glsl/shaders/painter/fastuidraw_painter_main.frag.glsl.resource_string 
    src/fastuidraw/glsl/shaders/painter/fastuidraw_painter_stroke_util.vert.glsl.resource_string 
    src/fastuidraw/glsl/shaders/painter/fastuidraw_painter_stroke.vert.glsl.resource_string 
    src/fastuidraw/glsl/shaders/painter/fastuidraw_painter_stroke.frag.glsl.resource_string 
    src/fastuidraw/glsl/shaders/painter/fastuidraw_painter_fill.vert.glsl.resource_string 
    src/fastuidraw/glsl/shaders/painter/fastuidraw_painter_fill.frag.glsl.resource_string 
    src/fastuidraw/glsl/shaders/painter/fastuidraw_painter_fill_aa_fuzz.vert.glsl.resource_string 
    src/fastuidraw/glsl/shaders/painter/fastuidraw_painter_fill_aa_fuzz.frag.glsl.resource_string
	src/fastuidraw/glsl/shaders/painter/fastuidraw_painter_stroke_util.frag.glsl.resource_string
	src/fastuidraw/glsl/shaders/painter/fastuidraw_painter_stroke_util.constants.glsl.resource_string

    #blend
    src/fastuidraw/glsl/shaders/painter/blend/framebuffer_fetch/fastuidraw_fbf_porter_duff_clear.glsl.resource_string
	src/fastuidraw/glsl/shaders/painter/blend/framebuffer_fetch/fastuidraw_fbf_porter_duff_dst_out.glsl.resource_string
	src/fastuidraw/glsl/shaders/painter/blend/framebuffer_fetch/fastuidraw_fbf_porter_duff_src_in.glsl.resource_string
	src/fastuidraw/glsl/shaders/painter/blend/framebuffer_fetch/fastuidraw_fbf_porter_duff_dst_atop.glsl.resource_string
	src/fastuidraw/glsl/shaders/painter/blend/framebuffer_fetch/fastuidraw_fbf_porter_duff_dst_over.glsl.resource_string
	src/fastuidraw/glsl/shaders/painter/blend/framebuffer_fetch/fastuidraw_fbf_porter_duff_src_out.glsl.resource_string
	src/fastuidraw/glsl/shaders/painter/blend/framebuffer_fetch/fastuidraw_fbf_porter_duff_dst.glsl.resource_string
	src/fastuidraw/glsl/shaders/painter/blend/framebuffer_fetch/fastuidraw_fbf_porter_duff_src_atop.glsl.resource_string
	src/fastuidraw/glsl/shaders/painter/blend/framebuffer_fetch/fastuidraw_fbf_porter_duff_src_over.glsl.resource_string
	src/fastuidraw/glsl/shaders/painter/blend/framebuffer_fetch/fastuidraw_fbf_porter_duff_dst_in.glsl.resource_string
	src/fastuidraw/glsl/shaders/painter/blend/framebuffer_fetch/fastuidraw_fbf_porter_duff_src.glsl.resource_string
	src/fastuidraw/glsl/shaders/painter/blend/framebuffer_fetch/fastuidraw_fbf_porter_duff_xor.glsl.resource_string

    src/fastuidraw/glsl/shaders/painter/blend/dual_src/fastuidraw_porter_duff_clear.glsl.resource_string
    src/fastuidraw/glsl/shaders/painter/blend/dual_src/fastuidraw_porter_duff_dst_out.glsl.resource_string
    src/fastuidraw/glsl/shaders/painter/blend/dual_src/fastuidraw_porter_duff_src_in.glsl.resource_string
    src/fastuidraw/glsl/shaders/painter/blend/dual_src/fastuidraw_porter_duff_dst_atop.glsl.resource_string
    src/fastuidraw/glsl/shaders/painter/blend/dual_src/fastuidraw_porter_duff_dst_over.glsl.resource_string
    src/fastuidraw/glsl/shaders/painter/blend/dual_src/fastuidraw_porter_duff_src_out.glsl.resource_string
    src/fastuidraw/glsl/shaders/painter/blend/dual_src/fastuidraw_porter_duff_dst.glsl.resource_string
    src/fastuidraw/glsl/shaders/painter/blend/dual_src/fastuidraw_porter_duff_src_atop.glsl.resource_string
    src/fastuidraw/glsl/shaders/painter/blend/dual_src/fastuidraw_porter_duff_src_over.glsl.resource_string
    src/fastuidraw/glsl/shaders/painter/blend/dual_src/fastuidraw_porter_duff_dst_in.glsl.resource_string
    src/fastuidraw/glsl/shaders/painter/blend/dual_src/fastuidraw_porter_duff_src.glsl.resource_string
    src/fastuidraw/glsl/shaders/painter/blend/dual_src/fastuidraw_porter_duff_xor.glsl.resource_string

    src/fastuidraw/glsl/shaders/painter/blend/single_src/fastuidraw_fall_through.glsl.resource_string

    #brush
    src/fastuidraw/glsl/shaders/painter/brush/fastuidraw_painter_brush.frag.glsl.resource_string
    src/fastuidraw/glsl/shaders/painter/brush/fastuidraw_painter_brush.vert.glsl.resource_string
    src/fastuidraw/glsl/shaders/painter/brush/fastuidraw_painter_brush_macros.glsl.resource_string
    src/fastuidraw/glsl/shaders/painter/brush/fastuidraw_painter_brush_types.glsl.resource_string
    src/fastuidraw/glsl/shaders/painter/brush/fastuidraw_painter_brush_unpack.glsl.resource_string
    src/fastuidraw/glsl/shaders/painter/brush/fastuidraw_painter_brush_unpack_forward_declares.glsl.resource_string

    #glyph
    src/fastuidraw/glsl/shaders/painter/glyph/fastuidraw_painter_glyph_coverage.frag.glsl.resource_string
    src/fastuidraw/glsl/shaders/painter/glyph/fastuidraw_painter_glyph_coverage.vert.glsl.resource_string
    src/fastuidraw/glsl/shaders/painter/glyph/fastuidraw_painter_glyph_curve_pair.frag.glsl.resource_string
    src/fastuidraw/glsl/shaders/painter/glyph/fastuidraw_painter_glyph_curve_pair.vert.glsl.resource_string
    src/fastuidraw/glsl/shaders/painter/glyph/fastuidraw_painter_glyph_curve_pair_anisotropic.frag.glsl.resource_string
    src/fastuidraw/glsl/shaders/painter/glyph/fastuidraw_painter_glyph_distance_field.frag.glsl.resource_string
    src/fastuidraw/glsl/shaders/painter/glyph/fastuidraw_painter_glyph_distance_field.vert.glsl.resource_string
    src/fastuidraw/glsl/shaders/painter/glyph/fastuidraw_painter_glyph_distance_field_anisotropic.frag.glsl.resource_string
)

set (PRIVATE_SOURCES
    ${PRIVATE_SRC_DIR}/interval_allocator.cpp
    ${PRIVATE_SRC_DIR}/path_util_private.cpp
    ${PRIVATE_SRC_DIR}/clip.cpp
    ${PRIVATE_SRC_DIR}/int_path.cpp
)

set (TEXT_SOURCES
    ${TEXT_SRC_DIR}/glyph_atlas.cpp
    ${TEXT_SRC_DIR}/glyph_render_data.cpp
    ${TEXT_SRC_DIR}/glyph_render_data_curve_pair.cpp
    ${TEXT_SRC_DIR}/glyph_render_data_distance_field.cpp
    ${TEXT_SRC_DIR}/glyph_render_data_coverage.cpp
    ${TEXT_SRC_DIR}/glyph_cache.cpp
    ${TEXT_SRC_DIR}/glyph_selector.cpp
    ${TEXT_SRC_DIR}/freetype_face.cpp
    ${TEXT_SRC_DIR}/freetype_lib.cpp
    ${TEXT_SRC_DIR}/font_freetype.cpp
    ${TEXT_SRC_DIR}/font_properties.cpp
    ${TEXT_SRC_DIR}/private/rect_atlas.cpp
)

set (UTIL_SOURCES
    ${UTIL_SRC_DIR}/static_resource.cpp
    ${UTIL_SRC_DIR}/fastuidraw_memory.cpp
    ${UTIL_SRC_DIR}/util.cpp
    ${UTIL_SRC_DIR}/blend_mode.cpp
    ${UTIL_SRC_DIR}/reference_count_mutex.cpp
    ${UTIL_SRC_DIR}/reference_count_atomic.cpp
    ${UTIL_SRC_DIR}/pixel_distance_math.cpp
    ${UTIL_SRC_DIR}/data_buffer.cpp
    ${UTIL_SRC_DIR}/api_callback.cpp
)

set (FASTUIDRAW_SOURCES
    ${FASTUIDRAW_SRC_DIR}/image.cpp
    ${FASTUIDRAW_SRC_DIR}/colorstop.cpp
    ${FASTUIDRAW_SRC_DIR}/colorstop_atlas.cpp
    ${FASTUIDRAW_SRC_DIR}/path.cpp
    ${FASTUIDRAW_SRC_DIR}/tessellated_path.cpp
)

set(GENERATED_SOURCE "")
foreach(shader IN LISTS SHADERS PAINTER_SHADERS)
    #message(STATUS "shader: ${shader}")
    add_shader_source(${shader})
    list(APPEND GENERATED_SOURCES ${FASTUIDRAW_RES_DIR}/${shader}.cpp)
endforeach(shader)

add_project(FastUIDraw STATIC "${GLU_TESS_SOURCES};${GLSL_SOURCES};${PRIVATE_SOURCES};${PAINTER_SOURCES};${GENERATED_SOURCES};${TEXT_SOURCES};${UTIL_SOURCES};${FASTUIDRAW_SOURCES}")
target_link_libraries(FastUIDraw ${FT_LIB})
target_include_directories(FastUIDraw PUBLIC "${FT_INC_DIR}")
if (MSVC)
    target_compile_definitions(FastUIDraw PRIVATE
            #"FASTUIDRAW_API=__declspec(dllexport)"
            "FASTUIDRAW_API"
            )
endif ()

set (GL_SOURCES
    ${GL_BACKEND_SRC_DIR}/gl_get.cpp
    ${GL_BACKEND_SRC_DIR}/opengl_trait.cpp
    ${GL_BACKEND_SRC_DIR}/gluniform_implement.cpp
    ${GL_BACKEND_SRC_DIR}/gl_program.cpp
    ${GL_BACKEND_SRC_DIR}/gl_context_properties.cpp
    ${GL_BACKEND_SRC_DIR}/image_gl.cpp
    ${GL_BACKEND_SRC_DIR}/colorstop_atlas_gl.cpp
    ${GL_BACKEND_SRC_DIR}/glyph_atlas_gl.cpp
    ${GL_BACKEND_SRC_DIR}/painter_backend_gl.cpp
    ${GL_BACKEND_SRC_DIR}/private/tex_buffer.cpp
    ${GL_BACKEND_SRC_DIR}/private/texture_gl.cpp
    ${GL_BACKEND_SRC_DIR}/private/texture_view.cpp
    #${FASTUIDRAW_RES_DIR}/gl_backend/shaders/fastuidraw_painter_gles_precision.glsl.resource_string.cpp
)
add_project(FastUIDrawGL STATIC "${GL_SOURCES}")
target_link_libraries(FastUIDrawGL "-WHOLEARCHIVE:$<TARGET_FILE:FastUIDraw>" NGL)
if (MSVC)
    target_compile_definitions(FastUIDrawGL PRIVATE
        #"FASTUIDRAW_GL_API=__declspec(dllexport)"
        "FASTUIDRAW_GL_API"
        )
    target_compile_definitions(FastUIDrawGL PUBLIC
            #"FASTUIDRAW_API=__declspec(dllimport)"
            #"FASTUIDRAW_NGL_API=__declspec(dllimport)"
            "FASTUIDRAW_API"
            "FASTUIDRAW_NGL_API"
            #"GLAPI=__declspec(dllimport)"
            )
endif ()

set (COMMON_DEMO_SOURCES
	${TOP_DEMOS_DIR}/common/generic_command_line.cpp
	${TOP_DEMOS_DIR}/common/colorstop_command_line.cpp
	${TOP_DEMOS_DIR}/common/sdl_benchmark.cpp
	${TOP_DEMOS_DIR}/common/sdl_demo.cpp
	${TOP_DEMOS_DIR}/common/sdl_painter_demo.cpp
	${TOP_DEMOS_DIR}/common/PanZoomTracker.cpp
	${TOP_DEMOS_DIR}/common/ImageLoader.cpp
	${TOP_DEMOS_DIR}/common/read_colorstops.cpp
	${TOP_DEMOS_DIR}/common/read_path.cpp
	${TOP_DEMOS_DIR}/common/text_helper.cpp
	${TOP_DEMOS_DIR}/common/PainterWidget.cpp
	${TOP_DEMOS_DIR}/common/cycle_value.cpp
	${TOP_DEMOS_DIR}/common/random.cpp
	${TOP_DEMOS_DIR}/common/read_dash_pattern.cpp
    ${TOP_DEMOS_DIR}/common/egl_helper.cpp
	${TOP_DEMOS_DIR}/common/stream_holder.cpp
    )
add_object_project(demo-common "${COMMON_DEMO_SOURCES}")
target_include_directories(demo-common PUBLIC
    "${SDL2_DIR}/include"
    "${SDL2_IMAGE_DIR}/include"
    "${FT_INC_DIR}"
    "${EGL_INC_DIR}"
    )
add_dependencies(demo-common FastUIDrawGL)
#target_compile_definitions(demo-common PUBLIC
    #"FASTUIDRAW_API=__declspec(dllimport)"
    #"FASTUIDRAW_NGL_API=__declspec(dllimport)"
    #"FASTUIDRAW_GL_API=__declspec(dllimport)"
    #"GLAPI=__declspec(dllimport)"
    #)

set (PAINTER_CELLS_SOURCES
    ${TOP_DEMOS_DIR}/painter_cells/main.cpp
    ${TOP_DEMOS_DIR}/painter_cells/cell.cpp
    ${TOP_DEMOS_DIR}/painter_cells/table.cpp
    ${TOP_DEMOS_DIR}/painter_cells/cell_group.cpp
    )
add_demo_project(painter-cells "${PAINTER_CELLS_SOURCES}" "")
add_demo_project(painter-path-test "${TOP_DEMOS_DIR}/painter_path_test/main.cpp" "")
add_demo_project(image-test "${TOP_DEMOS_DIR}/atlas_tests/image_test/main.cpp" "")

set (gradient_test_shaders
    demos/atlas_tests/gradient_test/shaders/linear_gradient.vert.glsl.resource_string
    demos/atlas_tests/gradient_test/shaders/linear_gradient.frag.glsl.resource_string
    demos/atlas_tests/gradient_test/shaders/draw_pt.vert.glsl.resource_string
    demos/atlas_tests/gradient_test/shaders/draw_pt.frag.glsl.resource_string
)
add_demo_project(gradient-test "${TOP_DEMOS_DIR}/atlas_tests/gradient_test/main.cpp" "${gradient_test_shaders}")

set (glyph_test_shaders
    demos/atlas_tests/glyph_test/shaders/glyph.vert.glsl.resource_string
    demos/atlas_tests/glyph_test/shaders/coverage_glyph.frag.glsl.resource_string
    demos/atlas_tests/glyph_test/shaders/distance_glyph.frag.glsl.resource_string
    demos/atlas_tests/glyph_test/shaders/curvepair_glyph.frag.glsl.resource_string
    demos/atlas_tests/glyph_test/shaders/glyph_atlas.vert.glsl.resource_string
    demos/atlas_tests/glyph_test/shaders/glyph_atlas.frag.glsl.resource_string
    demos/atlas_tests/glyph_test/shaders/perform_aa.frag.glsl.resource_string
    demos/atlas_tests/glyph_test/shaders/gles_prec.frag.glsl.resource_string
)
add_demo_project(glyph-test "${TOP_DEMOS_DIR}/atlas_tests/glyph_test/main.cpp" "${glyph_test_shaders}")

set (C_SOURCES
    ${FASTUIDRAW_SRC_DIR}/c/colorstop_atlas_gl.cpp
    ${FASTUIDRAW_SRC_DIR}/c/colorstop_atlas_gl_params.cpp
    ${FASTUIDRAW_SRC_DIR}/c/configuration_gl.cpp
    ${FASTUIDRAW_SRC_DIR}/c/gl_binding.cpp
    ${FASTUIDRAW_SRC_DIR}/c/globals_private.cpp
    ${FASTUIDRAW_SRC_DIR}/c/glyph_atlas_gl.cpp
    ${FASTUIDRAW_SRC_DIR}/c/glyph_atlas_gl_params.cpp
    ${FASTUIDRAW_SRC_DIR}/c/image.cpp
    ${FASTUIDRAW_SRC_DIR}/c/image_atlas_gl.cpp
    ${FASTUIDRAW_SRC_DIR}/c/image_atlas_gl_params.cpp
    #${FASTUIDRAW_SRC_DIR}/c/image_loader.c
    ${FASTUIDRAW_SRC_DIR}/c/painter.cpp
    ${FASTUIDRAW_SRC_DIR}/c/painter_backend.cpp
    ${FASTUIDRAW_SRC_DIR}/c/painter_backend_gl.cpp
    ${FASTUIDRAW_SRC_DIR}/c/painter_brush.cpp
    ${FASTUIDRAW_SRC_DIR}/c/painter_dashed_stroke_params.cpp
    ${FASTUIDRAW_SRC_DIR}/c/painter_stroke_params.cpp
    ${FASTUIDRAW_SRC_DIR}/c/path.cpp
)
add_project(FastUIDrawC STATIC "${C_SOURCES}")
target_include_directories(FastUIDrawC PUBLIC ${TOP_INC_DIR})

set (TESTC_SOURCES
    ${FASTUIDRAW_SRC_DIR}/c/test.c
    #${FASTUIDRAW_SRC_DIR}/c/image_loader.c
)

add_demo_project(testC "${TESTC_SOURCES}" "")
target_link_libraries(testC FastUIDrawC)
if (WIN32)
    target_link_libraries(testC "opengl32.lib")
endif()

if (MSVC)
    set (MSVC_MACROS
        "NOMINMAX" #not define min/max, or std::min and std::max do not work
        "_USE_MATH_DEFINES" #for math constants M_PI, M_E, etc.
        )
    set (MSVC_OPTIONS
        #"/Za" #disable extensions, such that it recognizes "and, or, etc."
        #"/showIncludes" #for easier debugging
        )

    foreach(pro ${ALL_PROJECTS})
        target_compile_options (${pro} PUBLIC ${MSVC_OPTIONS})
        target_compile_definitions(${pro} PUBLIC ${MSVC_MACROS})
    endforeach()
endif (MSVC)

