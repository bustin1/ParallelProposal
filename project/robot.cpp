

#include <math.h>
#include <iostream>
#include "include/robot.h"


// yawRate := how much the robot can turn in radians
Robot::Robot() : yawRate(.5) { 

}

Robot::~Robot() {
    if (this->rays) {
        delete this->rays;
    }
}

void Robot::init(const int numRays, 
                 Grid* grid, 
                 const int gridScale,
                 int pos) {

    this->gridScale = gridScale;
    this->numRays = numRays; 
    this->imgWidth = grid->width * gridScale;
    this->rays = new int[numRays];
    this->grid = grid;
    this->robotScale = gridScale / 5; // heuristic?
    this->hitWall = false;

    int gridSize = grid->num_closed + grid->num_open;
    this->visitedStates = new bool[gridSize];
    for (int i=0; i<gridSize; i++) {
        this->visitedStates[i] = (grid->walls[i] == 1);
    }

    if (pos < 0) {
        // top left
        this->pos = 1.5f*gridScale + imgWidth * (grid->height-1) * gridScale;
    } else {
        this->pos = pos;
    }

    this->angle = rand_angle();
}

int* Robot::get_rays() {
    return this->rays;
}

int Robot::get_pos() {
    return this->pos;
}

double Robot::get_angle() {
    return this->angle;
}

double Robot::get_yaw_rate() {
    return this->yawRate;
}

int Robot::get_scale() {
    return this->robotScale;
}

void Robot::set_pose(int pos, double angle) {
    this->pos = pos;
    this->angle = angle;
}


void Robot::update_visited_states(int pos) {
    int gPos = to_grid(this->imgWidth, pos, this->gridScale);
    visitedStates[gPos] = true;
}

bool Robot::visited(int rayPos) {
    int gPos = to_grid(this->imgWidth, rayPos, this->gridScale);
    return visitedStates[gPos];
}

// turn dtheta degrees from it's own angle and move with it's speed
// note that dtheta must be clamped with yaw rate
bool Robot::move(double dtheta, double speed) {

    double candidate_angle = double_mod(this->angle + dtheta, 2 * PI);

    double dirX = speed * cos(candidate_angle);
    double dirY = speed * sin(candidate_angle);
    
    int rx = pos % this->imgWidth;
    int ry = pos / this->imgWidth;

    int candidate_x = round(rx + dirX);
    int candidate_y = round(ry + dirY);

    int candidate_pos = candidate_x + this->imgWidth*candidate_y;

    this->angle = candidate_angle;

    if (!grid->is_wall_at(to_grid(this->imgWidth, candidate_pos, this->gridScale))) {
        int prev = to_grid(this->imgWidth, this->pos, this->gridScale);
        int next= to_grid(this->imgWidth, candidate_pos, this->gridScale);
        if (prev != next) {
            update_visited_states(this->pos);
        }
        this->pos = candidate_pos;
        return false;
    }
    
    // if hit wall, don't move 
    return true;


}

void Robot::move_greedy(double& dtheta, double& speed) {


    int bestRayInd = 0;
    double maxRayLen = 0;

    if (hitWall) {
        // 1) find best ray
        for (int i=0; i<this->numRays; i++) {

            double rayLen = dist2d(this->rays[i], this->get_pos(), this->imgWidth);

            // need to interpolate to the weird bug with line intersections
            int ray = interpolate(this->get_pos(), this->rays[i], this->imgWidth, .95);

            if (maxRayLen < rayLen && !this->visited(ray)) {
                bestRayInd = i;
                maxRayLen = rayLen;
            }
        }

        if (maxRayLen == 0) {
            for (int i=0; i<this->numRays; i++) {
                double rayLen = dist2d(this->rays[i], this->get_pos(), this->imgWidth);
                if (maxRayLen < rayLen) {
                    bestRayInd = i;
                    maxRayLen = rayLen;
                }
            }
        }

        // 2) find closest unvisted state
        int start = get_pos();
        int nextGrid = to_grid(this->imgWidth, rays[bestRayInd], this->gridScale);
        nextGrid = to_img(this->grid->width, nextGrid, this->gridScale);

        int nextX = get_x(nextGrid, this->imgWidth) + this->gridScale/2;
        int nextY = get_y(nextGrid, this->imgWidth) + this->gridScale/2;

        int startX = get_x(start, this->imgWidth);
        int startY = get_y(start, this->imgWidth);

        double angle_dir = atan2((double) (nextY-startY), (double) (nextX-startX));
        dtheta = angle_dir - this->angle;

    }


    // 3) take that direction
    speed = 2;
    this->hitWall = move(dtheta, speed);

}

void Robot::print_stuff() {
    std::vector<std::string> chatter;
    chatter.push_back("I AM SPEED!!");
    chatter.push_back("Beep Boop");
    chatter.push_back("Rob");
    chatter.push_back("zzpf ... easy");
    std::cout << chatter[(int)chatter.size() * rand_num()] << "\n";

}












