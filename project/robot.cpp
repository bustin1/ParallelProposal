

#include <math.h>

#include "robot.h"
#include "util.h"

Robot::Robot(Grid* g, int gscale, int p) : yawRate (.01) {
    grid = g;
    pos = p;
    gridScale = gscale;
    angle = 0; // TODO: randomize angle, remove p from arguments
    speed = 0;
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


// turn dtheta degrjes from it's own angle and move with it's speed
void Robot::move(float dtheta, float speed) {

    angle += dtheta;
    float dirX = speed * cos(angle);
    float dirY = speed * sin(angle);
    
    int imgWidth = grid->width * gridScale;
    int rx = get_x(imgWidth);
    int ry = get_y(imgWidth);

    int candidate_x = round(rx + dirX);
    int candidate_y = round(ry + dirY);

    int candidate_pos = candidate_x+imgWidth*candidate_y;

    // if hit wall, don't move 
    if (!grid->is_wall_at(to_grid(imgWidth, candidate_pos, gridScale))) {
        pos = candidate_pos;  
    }


}














