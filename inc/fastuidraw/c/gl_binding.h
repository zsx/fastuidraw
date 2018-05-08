#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void fui_gl_binding_get_proc_function(void *(*get_proc)(const char* name), int fetch_functions);

#ifdef __cplusplus
}
#endif
