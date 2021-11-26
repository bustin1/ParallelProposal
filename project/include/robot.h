

#ifndef _ROBOT_H
#define _ROBOT_H

#include "grid.h"
#include "util.h"

class Robot {

private:

    Grid* grid;

    int robotScale;
    int numRays;
    int gridScale;
    int pos;
    int imgWidth;

    double speed;
    double angle;

    const double yawRate; // how fast the robot can turn per frame (in radians)

    int* rays;
    bool* visitedStates;

public:

    Robot();
    virtual ~Robot();

    bool move(double dtheta, double speed);
    void move_greedy(double& dtheta, double& speed);
    void set_pose(int p, double a);
    void init(const int rays, Grid* grid, int gridScale);
    void print_stuff();
    void update_visited_states();
    void init(const int numRays, 
                 Grid* grid, 
                 const int gridScale,
                 int pos);

    // getters and setters
    int get_pos();
    int get_scale();

    double get_angle();
    double get_speed();
    double get_yaw_rate();

    int* get_rays();

    bool visited(int pos);

};

#endif
