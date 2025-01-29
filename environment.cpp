#include "environment.h"
#include <cmath>

void Environment::initialize() {
    for (int lin = 0; lin < HEIGHT; lin++) {
        for (int col = 0; col < WIDTH; col++) {
            if (lin == 0 || lin == HEIGHT-1 || col == 0 || col == WIDTH-1) {
                grid[lin][col] = 2; // Border
            } else {
                grid[lin][col] = 0; // Empty space
            }
        }
    }
    
    const int OBSTACLE_SIZE = 16;
    const int OBSTACLE_POSITIONS[] = {25, 91, 160};
    
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            int lin = OBSTACLE_POSITIONS[i];
            int col = OBSTACLE_POSITIONS[j];
            
            for (int y = lin; y < lin + OBSTACLE_SIZE; y++) {
                for (int x = col; x < col + OBSTACLE_SIZE; x++) {
                    grid[y][x] = 2;
                }
            }
        }
    }
}

bool Environment::isPathClear(double startLin, double startCol, double angle, int steps) const {
    double testlin = startLin - (steps * sin((PI * angle) / 180));
    double testcol = startCol + (steps * cos((PI * angle) / 180));

    if (testlin < 1 || testlin > HEIGHT-2 || testcol < 1 || testcol > WIDTH-2) 
        return false;

    if (grid[(int)testlin][(int)testcol])
        return false;

    return true;
}

void Environment::setCell(int lin, int col, int value) {
    if (lin >= 0 && lin < HEIGHT && col >= 0 && col < WIDTH) {
        grid[lin][col] = value;
    }
}

int Environment::getCell(int lin, int col) const {
    if (lin >= 0 && lin < HEIGHT && col >= 0 && col < WIDTH) {
        return grid[lin][col];
    }
    return -1;
}