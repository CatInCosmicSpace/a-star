#ifndef A_STAR_MAZE_H
#define A_STAR_MAZE_H

#include "grid.h"
#include <algorithm>
#include "FastNoiseLite.h"

class maze {
public:
    void generate(int height, int width);
//    cell* getCell(int x, int y);
//    edge* getEdge(int x1, int y1, int x2, int y2);
private:
    grid map;
};


#endif //A_STAR_MAZE_H
