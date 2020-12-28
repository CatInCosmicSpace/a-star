#ifndef A_STAR_MAZE_H
#define A_STAR_MAZE_H

#include "grid.h"
#include <algorithm>
#include "FastNoiseLite.h"
#include "structs.h"

#include <iostream>
class maze {
public:
    void generate(int height, int width);
//    cell* getCell(int x, int y);
    float get_edge(Point from, Point to) {
        if (to.x < 0 || to.y < 0 || to.x >= _width || to.y >= _height)
            return 0.f;
        Vector dir = to - from;
        if (dir.x != 0) {
            int ind = dir.x == -1 ? from.x - 1 : from.x;
            ind += from.y * (_width - 1);
//            std::cout << "x " << ind << std::endl;
            return map.vertical[ind];
        } else {
            int ind = dir.y == -1 ? from.y - 1 : from.y;
            ind = ind * _width + from.x;
//            std::cout << "y " << ind << std::endl;
            return map.horizontal[ind];
        }
    }
    size_t width() {
        return _width;
    }
    size_t height() {
        return _height;
    }
private:
    size_t _width;
    size_t _height;
    grid map;
};


#endif //A_STAR_MAZE_H
