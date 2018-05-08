#include <turbojpeg.h>
#include <stdlib.h>

#include "fastuidraw/c/image.h"

unsigned char*load_jpeg(const unsigned char* jpeg, size_t len, int *w, int *h)
{
    long unsigned int compressed_size;
    unsigned char* compressed_data;

    int subsamp, width, height, colorspace;

    tjhandle hnd = tjInitDecompress();

    tjDecompressHeader3(hnd, jpeg, len, &width, &height, &subsamp, &colorspace);

    unsigned char *buffer = malloc(width * height * 4);
    tjDecompress2(hnd, jpeg, len, buffer, width, 0/*pitch*/, height, TJPF_RGBA, TJFLAG_FASTDCT);

    tjDestroy(hnd);

    if (w) *w = width;
    if (h) *h = height;

    return buffer;
}

void unload_jpeg(unsigned char *jpeg)
{
    free(jpeg);
}
