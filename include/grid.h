#ifndef A_STAR_GRID_H
#define A_STAR_GRID_H

#include <vector>
#include <map>
#include <set>
#include <utility>
#include <memory>
#include <cmath>

#define WALL 0.f

struct grid {
    std::vector<float> vertical;
    std::vector<float> horizontal;

    void generateWalls(int height, int width);
};


#endif //A_STAR_GRID_H
