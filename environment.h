#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

#define HEIGHT 200
#define WIDTH 200

class Environment {
private:
    int grid[HEIGHT][WIDTH];

public:
    void initialize();
    bool isPathClear(double startLin, double startCol, double angle, int steps) const;
    void setCell(int lin, int col, int value);
    int getCell(int lin, int col) const;
};

#endif // ENVIRONMENT_H