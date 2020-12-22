#include <iostream>
#include "maze.h"

int main(int argc, char* argv[]) {
    auto m = maze{};
    m.generate(200, 200);
    std::cout << "Hello world!" << std::endl;
    return 0;
}
