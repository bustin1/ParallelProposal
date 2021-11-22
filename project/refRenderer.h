
#ifndef __REF_RENDERER_H__
#define __REF_RENDERER_H__

#include "particleFilter.h"
#include "image.h"


/*
 * This is the rendered that defines the interface for the display to use
 * This involves creating an image, clearing the image, animation, and
 * rendering. It also needs to know the particle filter because it needs to
 * know where to draw the particles. The image is trivially defined here.
 */

class RefRenderer {

private:

    Pfilter* filter;
    Image* image;

public:

    RefRenderer(Pfilter* filter);
    virtual ~RefRenderer();

    const Image* getImage();

    void setup();

    void render();

    void clearImage();

    void advanceAnimation();

    void dumpWalls(const char* filename);

};


#endif
