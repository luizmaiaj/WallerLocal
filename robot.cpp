#include "robot.h"
#include <cmath>
#include <cstdlib>
#include <ctime>

#define ANGLE 5
#define VIEW_ANGLE 30

Robot::Robot(Environment& environment) : env(environment) {}

void Robot::initialize() {
    do {
        dir = ANGLE * (rand() % (360 / ANGLE));
        col = (int)(rand() % (WIDTH-2)) + 1;
        lin = (int)(rand() % (HEIGHT-2)) + 1;

        if (env.getCell((int)lin, (int)col)) {
            if (rand() % 2) {
                col = (int)(rand() % (WIDTH-2)) + 1;
            } else {
                lin = (int)(rand() % (HEIGHT-2)) + 1;
            }
        }
    } while (env.getCell((int)lin, (int)col));
    
    env.setCell((int)lin, (int)col, 1);
}

void Robot::walkFront() {
    double angle = dir;
    double testlin = lin - sin((PI * angle) / 180);
    double testcol = col + cos((PI * angle) / 180);

    if (!env.getCell((int)testlin, (int)testcol)) {
        env.setCell((int)lin, (int)col, 0);
        lin = testlin;
        col = testcol;
        env.setCell((int)lin, (int)col, 1);
    }
}

void Robot::walkBack() {
    double angle = dir + 180;
    double testlin = lin - sin((PI * angle) / 180);
    double testcol = col + cos((PI * angle) / 180);

    if (!env.getCell((int)testlin, (int)testcol)) {
        env.setCell((int)lin, (int)col, 0);
        lin = testlin;
        col = testcol;
        env.setCell((int)lin, (int)col, 1);
    }
}

void Robot::turnLeft() {
    dir += ANGLE;
    if (dir >= 360) {
        dir -= 360;
    }
}

void Robot::turnRight() {
    dir -= ANGLE;
    if (dir < 0) {
        dir += 360;
    }
}

double normalizeAngle(double angle) {
    while (angle >= 360) angle -= 360;
    while (angle < 0) angle += 360;
    return angle;
}

double calculateAngleBetweenPoints(double y1, double x1, double y2, double x2) {
    double dy = y2 - y1;
    double dx = x2 - x1;
    double angle = (180 / PI) * atan2(dy, dx);
    
    if (dx >= 0)
        angle = 360 - angle;
    else
        angle = 180 - angle;
        
    return normalizeAngle(angle);
}

bool isAngleInRange(double angle1, double angle2, double range) {
    return abs((int)(angle1 - angle2)) <= range;
}

void Robot::align(double ballLin, double ballCol) {
    double angle = calculateAngleBetweenPoints(lin, col, ballLin, ballCol);
    
    if (isAngleInRange(angle, dir, VIEW_ANGLE)) {
        dir = normalizeAngle(dir);
        if (env.isPathClear(lin, col, angle, 1)) {
            dir = angle;
        }
    }
}

bool Robot::isNearWall() const {
    return !env.isPathClear(lin, col, dir, 2);
}

bool Robot::canSeeBall(double ballLin, double ballCol) const {
    double angle = calculateAngleBetweenPoints(lin, col, ballLin, ballCol);

    if ((int)(angle - dir) > VIEW_ANGLE || (int)(angle - dir) < -VIEW_ANGLE) {
        if (env.isPathClear(lin, col, angle, 1)) {
            return false;
        }
    }
    return true;
}