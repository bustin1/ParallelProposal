
#ifndef __REF_RENDERER_H__
#define __REF_RENDERER_H__

#include "particleFilter.h"
#include "image.h"

class RefRenderer {

private:

    Pfilter* filter;
    Image* image;
    int scale;

public:

    RefRenderer(Pfilter* filter, int scale);
    virtual ~RefRenderer();

    const Image* getImage();

    void setup();

    void allocOutputImage(int scale);

    void render();

    void clearImage();

    void advanceAnimation();

    void dumpWalls(const char* filename);

};


#endif
