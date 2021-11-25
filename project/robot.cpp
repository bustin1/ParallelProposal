

#include <math.h>

#include "robot.h"
#include "util.h"


Robot::Robot(Grid* g, int gscale) : yawRate (.25), robotScale (10) {
    grid = g;
    gridScale = gscale;
}


Robot::Robot(Grid* g, int gscale, int p, double a) : yawRate (.25), robotScale (10) {
    grid = g;
    pos = p;
    gridScale = gscale;
    angle = a; // TODO: randomize angle, remove p from arguments
}

Robot::~Robot() {
    printf("deleting robot beep boop ...");
    if (rays) {
        delete rays;
    }
}

void Robot::init_rays(const int numRays) {
    rays = new int[numRays];
}

int* Robot::get_rays() {
    return rays;
}

int Robot::get_pos() {
    return pos;
}

double Robot::get_angle() {
    return angle;
}

int Robot::get_x(int width) {
    return pos % width;
}

int Robot::get_y(int width) {
    return pos / width;
}

float Robot::get_yaw_rate() {
    return yawRate;
}

int Robot::get_scale() {
    return robotScale;
}

void Robot::set_pose(int p, double a) {
    pos = p;
    angle = a;
}


// turn dtheta degrjes from it's own angle and move with it's speed
// note that dtheta must be clamped with yaw rate
void Robot::move(double dtheta, double speed) {

    int imgWidth = grid->width * gridScale;

    int loc = pos;

    double candidate_angle = double_mod(angle + dtheta, 2 * PI);

    double dirX = speed * cos(candidate_angle);
    double dirY = speed * sin(candidate_angle);
    
    int rx = pos % imgWidth;
    int ry = pos / imgWidth;

    int candidate_x = round(rx + dirX);
    int candidate_y = round(ry + dirY);

    int candidate_pos = candidate_x+imgWidth*candidate_y;

    // if hit wall, don't move 
    if (!grid->is_wall_at(to_grid(imgWidth, candidate_pos, gridScale))) {
        loc = candidate_pos;  
    }

    angle = candidate_angle;
    pos = loc;

}














