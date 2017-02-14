# Begin standard header
sp 		:= $(sp).x
dirstack_$(sp)	:= $(d)
d		:= $(dir)
# End standard header

LIBRARY_PRIVATE_SOURCES += $(call filelist, GrTessellator.cpp)

# Begin standard footer
d		:= $(dirstack_$(sp))
sp		:= $(basename $(sp))
# End standard footer
