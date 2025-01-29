#ifndef ROBOT_H
#define ROBOT_H

#include "environment.h"

class Robot {
private:
    double lin;
    double col;
    int dir;
    Environment& env;

public:
    Robot(Environment& environment);
    void initialize();
    void walkFront();
    void walkBack();
    void turnLeft();
    void turnRight();
    void align(double ballLin, double ballCol);
    bool isNearWall() const;
    bool canSeeBall(double ballLin, double ballCol) const;
    bool canSeeAndReachBall(double ballLin, double ballCol, double angle) const;
    
    // Getters
    double getLine() const { return lin; }
    double getColumn() const { return col; }
    int getDirection() const { return dir; }
    
    // For moveball compatibility
    friend void moveball(struct ball_data* ball, const Robot& robot);
};

#endif // ROBOT_H