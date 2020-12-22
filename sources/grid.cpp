#include "grid.h"

void grid::generateWalls(int height, int width) {
    this->vertical.resize((width - 1) * height, WALL);
    this->horizontal.resize((height - 1) * width, WALL);
}
