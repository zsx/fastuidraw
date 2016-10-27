# Begin standard header
sp 		:= $(sp).x
dirstack_$(sp)	:= $(d)
d		:= $(dir)
# End standard header

LIBRARY_SOURCES += $(call filelist, \
	stroked_path.cpp filled_path.cpp \
	painter_attribute_data_filler_path_fill.cpp \
	painter_stroke_params.cpp \
	painter_dashed_stroke_params.cpp \
	painter_dashed_stroke_shader_set.cpp \
	painter_stroke_shader.cpp \
	painter_fill_shader.cpp)

# Begin standard footer
d		:= $(dirstack_$(sp))
sp		:= $(basename $(sp))
# End standard footer
