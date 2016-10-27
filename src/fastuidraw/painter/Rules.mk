# Begin standard header
sp 		:= $(sp).x
dirstack_$(sp)	:= $(d)
d		:= $(dir)
# End standard header


dir := $(d)/packing
include $(dir)/Rules.mk

dir := $(d)/blend
include $(dir)/Rules.mk

dir := $(d)/brush
include $(dir)/Rules.mk

dir := $(d)/text
include $(dir)/Rules.mk

dir := $(d)/paths
include $(dir)/Rules.mk

LIBRARY_SOURCES += $(call filelist, \
	painter_attribute_data.cpp \
	painter.cpp painter_enums.cpp \
	painter_shader_data.cpp \
	painter_shader.cpp painter_shader_set.cpp)

# Begin standard footer
d		:= $(dirstack_$(sp))
sp		:= $(basename $(sp))
# End standard footer
