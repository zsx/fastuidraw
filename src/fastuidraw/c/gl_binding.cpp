#include "fastuidraw/gl_backend/gl_binding.hpp"
#include "fastuidraw/c/gl_binding.h"


void fui_gl_binding_get_proc_function(void *(*get_proc)(const char* name), int fetch_functions)
{
    fastuidraw::gl_binding::get_proc_function(get_proc, fetch_functions);
}
