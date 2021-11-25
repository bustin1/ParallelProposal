

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
    int robotScale;
    int* rays;


public:

    Robot(Grid* grid, int gscale); // for random position

    virtual ~Robot();

    Robot(Grid* grid, int gscale, int pos, double angle);

    float get_speed();

    void move(double dtheta, double speed);

    float get_yaw_rate();

    int get_pos();

    double get_angle();

    int get_scale();

    int get_x(int width);

    int get_y(int width);

    void set_pose(int p, double a);

    void init_rays(const int);

    int* get_rays();

};

#endif
