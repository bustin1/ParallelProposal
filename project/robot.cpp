

#include <math.h>
#include <iostream>
#include "robot.h"
#include "util.h"


Robot::Robot(Grid* g, int gscale) : yawRate (.25), robotScale (10) {
    grid = g;
    gridScale = gscale;
    int imgW = gscale * g->width;
    set_pose(1.5f*gridScale + imgW * (grid->height-1) * gridScale, 2.0f * PI * rand_num());
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

void Robot::init_rays(const int n) {
    numRays = n;
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
bool Robot::move(double dtheta, double speed) {

    int imgWidth = grid->width * gridScale;

    double candidate_angle = double_mod(angle + dtheta, 2 * PI);

    double dirX = speed * cos(candidate_angle);
    double dirY = speed * sin(candidate_angle);
    
    int rx = pos % imgWidth;
    int ry = pos / imgWidth;

    int candidate_x = round(rx + dirX);
    int candidate_y = round(ry + dirY);

    int candidate_pos = candidate_x+imgWidth*candidate_y;

    angle = candidate_angle;

    // if hit wall, don't move 
    if (!grid->is_wall_at(to_grid(imgWidth, candidate_pos, gridScale))) {
        pos = candidate_pos;
        return false;
    }
    
    return true;


}

void Robot::move_greedy(double& dtheta, double speed) {

    int imgWidth = grid->width * gridScale;

    // 1) find max ray length

    double maxRay = 0;
    int maxRayInd = 0;
    for (int i=0; i<numRays; i++) {
        double rayLen = dist2d(pos, rays[i], imgWidth);
        if (maxRay < rayLen) {
            maxRay = rayLen;
            maxRayInd = i;
        }
    }

    // 2) travese that direction

    double sx = get_x(imgWidth);
    double sy = get_y(imgWidth);
    double ex = rays[maxRayInd] % imgWidth;
    double ey = rays[maxRayInd] / imgWidth;

    int dir_x = round((ex - sx) / maxRay * speed);
    int dir_y = round((ey - sy) / maxRay * speed);

    pos = imgWidth*(dir_y+sy) + (sx+dir_x);
    dtheta = atan2(ey-sy, ex-sx) - angle;
    angle += dtheta;

}

void Robot::print_stuff() {
    std::vector<std::string> chatter;
    chatter.push_back("I AM SPEED!!");
    chatter.push_back("Beep Boop");
    chatter.push_back("Rob");
    chatter.push_back("zzpf ... easy");
    std::cout << chatter[(int)chatter.size() * rand_num()] << "\n";

}












