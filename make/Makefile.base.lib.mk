FASTUIDRAW_DEPS_LIBS += $(shell freetype-config --libs)
FASTUIDRAW_DEPS_STATIC_LIBS += $(shell freetype-config --static --libs)

FASTUIDRAW_BASE_CFLAGS = -std=c++11 -D_USE_MATH_DEFINES
FASTUIDRAW_debug_BASE_CFLAGS = $(FASTUIDRAW_BASE_CFLAGS) -DFASTUIDRAW_DEBUG
FASTUIDRAW_release_BASE_CFLAGS = $(FASTUIDRAW_BASE_CFLAGS)

FASTUIDRAW_DEPENDS_CFLAGS = $(shell freetype-config --cflags)
FASTUIDRAW_debug_CFLAGS =  $(FASTUIDRAW_DEPENDS_CFLAGS) $(FASTUIDRAW_debug_BASE_CFLAGS)
FASTUIDRAW_release_CFLAGS = $(FASTUIDRAW_DEPENDS_CFLAGS) $(FASTUIDRAW_release_BASE_CFLAGS)

FASTUIDRAW_BUILD_debug_FLAGS = -g
FASTUIDRAW_BUILD_release_FLAGS = -O3 -fstrict-aliasing
FASTUIDRAW_BUILD_WARN_FLAGS = -Wall -Wextra -Wcast-qual -Wwrite-strings
FASTUIDRAW_BUILD_INCLUDES_CFLAGS = -Iinc

#
#  STRING_RESOURCE_CC inputfile resourcename outputpath
#    inputfile has full path on it
#    resourcename name to access string resource
#    outputpath is path to place outputpath
build/string_resources_cpp/%.resource_string.cpp: %.resource_string $(STRING_RESOURCE_CC)
	@mkdir -p $(dir $@)
	$(STRING_RESOURCE_CC) $< $(notdir $<) $(dir $@)

FASTUIDRAW_STRING_RESOURCES_SRCS = $(patsubst %.resource_string, build/string_resources_cpp/%.resource_string.cpp, $(FASTUIDRAW_RESOURCE_STRING) )
CLEAN_FILES += $(FASTUIDRAW_STRING_RESOURCES_SRCS)

# $1 --> release or debug
define librules
$(eval FASTUIDRAW_$(1)_OBJS = $$(patsubst %.cpp, build/$(1)/%.o, $(FASTUIDRAW_SOURCES))
FASTUIDRAW_$(1)_DEPS = $$(patsubst %.cpp, build/$(1)/%.d, $(FASTUIDRAW_SOURCES))
FASTUIDRAW_$(1)_DEPS += $$(patsubst %.cpp, build/$(1)/private/%.d, $(FASTUIDRAW_PRIVATE_SOURCES))
FASTUIDRAW_PRIVATE_$(1)_OBJS = $$(patsubst %.cpp, build/$(1)/private/%.o, $(FASTUIDRAW_PRIVATE_SOURCES))
FASTUIDRAW_$(1)_RESOURCE_OBJS = $$(patsubst %.resource_string, build/$(1)/%.resource_string.o, $(FASTUIDRAW_RESOURCE_STRING))
FASTUIDRAW_$(1)_ALL_OBJS = $$(FASTUIDRAW_$(1)_OBJS) $$(FASTUIDRAW_PRIVATE_$(1)_OBJS) $$(FASTUIDRAW_$(1)_RESOURCE_OBJS)
COMPILE_$(1)_CFLAGS=$$(FASTUIDRAW_BUILD_$(1)_FLAGS) $(FASTUIDRAW_BUILD_WARN_FLAGS) $(FASTUIDRAW_BUILD_INCLUDES_CFLAGS) $$(FASTUIDRAW_$(1)_CFLAGS)
CLEAN_FILES += $$(FASTUIDRAW_$(1)_ALL_OBJS) $$(FASTUIDRAW_$(1)_RESOURCE_OBJS)
SUPER_CLEAN_FILES += $$(FASTUIDRAW_$(1)_DEPS)
build/$(1)/%.resource_string.o: build/string_resources_cpp/%.resource_string.cpp
	@mkdir -p $$(dir $$@)
	$(CXX) $$(COMPILE_$(1)_CFLAGS) $(fPIC) -c $$< -o $$@

build/$(1)/%.o: %.cpp build/$(1)/%.d
	@mkdir -p $$(dir $$@)
	$(CXX) $$(COMPILE_$(1)_CFLAGS) $(fPIC) -MT $$@ -MMD -MP -MF build/$(1)/$$*.d -c $$< -o $$@

build/$(1)/%.d: ;
.PRECIOUS: build/$(1)/%.d

build/$(1)/private/%.o: %.cpp build/$(1)/private/%.d
	@mkdir -p $$(dir $$@)
	$(CXX) $$(COMPILE_$(1)_CFLAGS) $(fPIC) $(fHIDDEN) -MT $$@ -MMD -MP -MF build/$(1)/private/$$*.d -c $$< -o $$@

build/$(1)/private/%.d: ;
.PRECIOUS: build/$(1)/%.d

TARGET_LIST += libFastUIDraw_$(1)
ifeq ($(MINGW_BUILD),1)

libFastUIDraw_$(1): libFastUIDraw_$(1).dll
libFastUIDraw_$(1).dll.a: libFastUIDraw_$(1).dll
libFastUIDraw_$(1).dll: $$(FASTUIDRAW_$(1)_ALL_OBJS)
	$(CXX) -shared -Wl,--out-implib,libFastUIDraw_$(1).dll.a -o libFastUIDraw_$(1).dll $$(FASTUIDRAW_$(1)_ALL_OBJS) $(FASTUIDRAW_DEPS_LIBS)
CLEAN_FILES += libFastUIDraw_$(1).dll libFastUIDraw_$(1).dll.a
INSTALL_LIBS += libFastUIDraw_$(1).dll.a
INSTALL_EXES += libFastUIDraw_$(1).dll

else

libFastUIDraw_$(1): libFastUIDraw_$(1).so
libFastUIDraw_$(1).so: $(FASTUIDRAW_STRING_RESOURCES_SRCS) $$(FASTUIDRAW_$(1)_ALL_OBJS)
	$(CXX) -shared -Wl,-soname,libFastUIDraw_$(1).so -o libFastUIDraw_$(1).so $$(FASTUIDRAW_$(1)_ALL_OBJS) $(FASTUIDRAW_DEPS_LIBS)
CLEAN_FILES += libFastUIDraw_$(1).so
INSTALL_LIBS += libFastUIDraw_$(1).so
.PHONY: libFastUIDraw_$(1) libFastUIDraw
endif

libFastUIDraw-static: libFastUIDraw_$(1)-static
.PHONY: libFastUIDraw-static
libFastUIDraw_$(1)-static: libFastUIDraw_$(1).a
.PHONY: libFastUIDraw_$(1)-static
libFastUIDraw_$(1).a: $(FASTUIDRAW_STRING_RESOURCES_SRCS) $$(FASTUIDRAW_$(1)_ALL_OBJS)
	ar rcs $$@ $$(FASTUIDRAW_$(1)_ALL_OBJS)
CLEAN_FILES += libFastUIDraw_$(1).a
TARGETLIST += libFastUIDraw_$(1)-static

-include $$(FASTUIDRAW_$(1)_DEPS)

libFastUIDraw: libFastUIDraw_$(1)
)
endef

TARGETLIST += libFastUIDraw libFastUIDraw-static
$(call librules,release)
$(call librules,debug)
all: libFastUIDraw

ifeq ($(BUILD_NEGL),1)
TARGETLIST += libNEGL
TARGETLIST += libNEGL_debug libNEGL_release
all: libNEGL
endif
