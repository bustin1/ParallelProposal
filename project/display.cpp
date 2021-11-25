
#include <algorithm>

#include "cycleTimer.h"
#include "grid.h"
#include "platformgl.h"
#include "refRenderer.h"

#include "debug.h"


void renderPicture();


static struct {
    int width;
    int height;
    bool updateSim;
    bool printStats;
    bool pauseSim;
    double lastFrameTime;

    RefRenderer* renderer;

} gDisplay;

// handleReshape --
//
// Event handler, fired when the window is resized
void handleReshape(int w, int h) {
    gDisplay.width = w;
    gDisplay.height = h;
    glViewport(0, 0, gDisplay.width, gDisplay.height);
    glutPostRedisplay();
}

void handleDisplay() {

    // simulation and rendering work is done in the renderPicture
    // function below

    renderPicture();

    // the subsequent code uses OpenGL to present the state of the
    // rendered image on the screen.

    const Image* img = gDisplay.renderer->get_image();

    int width = std::min(img->width, gDisplay.width);
    int height = std::min(img->height, gDisplay.height);

    glDisable(GL_DEPTH_TEST);
    glClearColor(0.f, 0.f, 0.f, 1.f);
    glClear(GL_COLOR_BUFFER_BIT);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0.f, gDisplay.width, 0.f, gDisplay.height, -1.f, 1.f);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    // copy image data from the renderer to the OpenGL
    // frame-buffer.  This is inefficient solution is the processing
    // to generate the image is done in CUDA.  An improved solution
    // would render to a CUDA surface object (stored in GPU memory),
    // and then bind this surface as a texture enabling it's use in
    // normal openGL rendering


    glRasterPos2i(0, 0);
    glDrawPixels(width, height, GL_RGBA, GL_FLOAT, img->data);
    if (DEBUG) {

        glColor3f(0, 1, 0);

        // draw rays (only debugging though)
        Pfilter* filter = gDisplay.renderer->get_filter();
        int numParticles = filter->get_num_particles();
        int numRays = filter->get_num_rays();
        int* pLoc = filter->get_particle_locations();
        int imageWidth = img->width;
        int* rays = filter->get_rays();


        for (int i=0; i<numParticles; i++) {
            if (DEBUG == 2) {
                i = filter->get_best_particle();
            }
            int loc = pLoc[i];
            int x1 = loc % imageWidth;
            int y1 = loc / imageWidth;
            for (int j=0; j<numRays; j++) {
                int x2 = rays[i*numRays+j] % imageWidth;
                int y2 = rays[i*numRays+j] / imageWidth;
                glBegin(GL_LINES);
                    glVertex2f(x1, y1);
                    glVertex2f(x2, y2);
                glEnd();
            }
            if (DEBUG == 2) {
                break;
            }
        }

        glColor3f(1, 0, 1);
        Robot* robot = filter->get_robot();
        int* roboRays = robot->get_rays();
        int x1 = robot->get_x(imageWidth);
        int y1 = robot->get_y(imageWidth);
        for (int j=0; j<numRays; j++) {
            int x2 = roboRays[j] % imageWidth;
            int y2 = roboRays[j] / imageWidth;
            glBegin(GL_LINES);
                glVertex2f(x1, y1);
                glVertex2f(x2, y2);
            glEnd();
        }

    }


    double currentTime = CycleTimer::currentSeconds();

    if (gDisplay.printStats)
        printf("%.2f ms\n", 1000.f * (currentTime - gDisplay.lastFrameTime));

    gDisplay.lastFrameTime = currentTime;

    glutSwapBuffers();
    glutPostRedisplay();
}


// handleKeyPress --
//
// Keyboard event handler
void
handleKeyPress(unsigned char key, int x, int y) {

    switch (key) {
    case 'q':
    case 'Q':
        exit(1);
        break;
    case '=':
    case '+':
        gDisplay.updateSim = true;
        break;
    case 'p':
    case 'P':
        gDisplay.pauseSim = !gDisplay.pauseSim;
        if (!gDisplay.pauseSim)
            gDisplay.updateSim = true;
        break;
    }
}

// renderPicture --
//
// At the reall work is done here, not in the display handler
void renderPicture() {

    double startTime = CycleTimer::currentSeconds();

    // clear screen
    gDisplay.renderer->clearImage();

    double endClearTime = CycleTimer::currentSeconds();

    // update particle positions and state
    if (gDisplay.updateSim) {
        gDisplay.renderer->advanceAnimation();
    }
    if (gDisplay.pauseSim)
        gDisplay.updateSim = false;

    double endSimTime = CycleTimer::currentSeconds();

    // render the particles< into the image
    gDisplay.renderer->render();

    double endRenderTime = CycleTimer::currentSeconds();

    if (gDisplay.printStats) {
        printf("Clear:    %.3f ms\n", 1000.f * (endClearTime - startTime));
        printf("Advance:  %.3f ms\n", 1000.f * (endSimTime - endClearTime));
        printf("Render:   %.3f ms\n", 1000.f * (endRenderTime - endSimTime));
    }
}

void startRendererWithDisplay(RefRenderer* renderer) {

    // setup the display

    const Image* img = renderer->get_image();

    gDisplay.renderer = renderer;
    gDisplay.updateSim = true;
    gDisplay.pauseSim = false;
    gDisplay.printStats = false;
    gDisplay.lastFrameTime = CycleTimer::currentSeconds();
    gDisplay.width = img->width;
    gDisplay.height = img->height;

    // configure GLUT

    glutInitWindowSize(gDisplay.width, gDisplay.height);
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
    glutCreateWindow("Particle Filtering");
    glutDisplayFunc(handleDisplay);
    glutKeyboardFunc(handleKeyPress);
    glutMainLoop();

}
