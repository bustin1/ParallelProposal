

#ifndef _ROBOT_H
#define _ROBOT_H

#include "grid.h"

class Robot {

private:

    Grid* grid;
    int gridScale;
    int pos;
    float yawRate; // how fast the robot can turn per frame (in radians)
    float speed;
    double angle;

    int get_x(int width);
    int get_y(int width);

public:

    Robot(Grid* grid, int gscale, int pos);

    float get_speed();

    void move(float dtheta, float speed);

    float get_yaw_rate();

    int get_pos();

    double get_angle();

};

#endif
