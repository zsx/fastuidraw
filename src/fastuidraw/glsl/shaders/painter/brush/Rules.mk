# Begin standard header
sp 		:= $(sp).x
dirstack_$(sp)	:= $(d)
d		:= $(dir)
# End standard header

dir := $(d)/fixed_function
include $(dir)/Rules.mk

LIBRARY_RESOURCE_STRING += $(call filelist, \
	fastuidraw_painter_brush_types.glsl.resource_string \
	fastuidraw_painter_brush_read_data.glsl.resource_string \
	fastuidraw_painter_brush_read_data_forward_declares.glsl.resource_string \
	)
# Begin standard footer
d		:= $(dirstack_$(sp))
sp		:= $(basename $(sp))
# End standard footer
