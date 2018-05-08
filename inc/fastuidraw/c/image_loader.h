#pragma once

unsigned char*load_jpeg(const unsigned char* jpeg, size_t len, int *w, int *h);
void unload_jpeg(unsigned char *jpeg);
