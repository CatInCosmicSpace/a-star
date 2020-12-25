#include "maze.h"
#include <iostream>
#include <unordered_set>
#include <queue>
#include <utility>
#include "structs.h"


std::vector<Vector> dir4 = {
        Point(1, 0),
        Point(0, 1),
        Point(-1, 0),
        Point(0, -1)
};

float data[][5] = {
        {1.0, 1.0, 1.0, 1.0, 1.0},
        {1.0, 0.0, 1.0, 1.0, 1.0},
        {1.0, 0.0, 1.0, 1.0, 1.0},
        {1.0, 0.0, 0.0, 0.0, 1.0},
        {1.0, 1.0, 1.0, 1.0, 1.0},
};

class TestMap : public Map {
public:
    size_t width() override {
        return 5;
    }
    size_t height() override {
        return 5;
    }
    Point start() override {
        return {4, 0};
    }
    Point goal() override {
        return {0, 4};
    }
    float get_edge(Point position, Vector direction) override {
        auto to = position + direction;
        if (to.x < 0 || to.y < 0 || to.x >= width() || to.y >= height())
            return 1.f; //std::_nanf();
        return data[to.y][to.x];// == 1.f ? std::_nanf() : data[to.y][to.x];
    }
    std::vector<Vector> directions() override {
        return dir4;
    }
};


class TestMapGenerator : public MapGenerator {
public:
    MapPtr generate() override {
        return std::make_shared<TestMap>();
    }
};


class TestAStar : public AStar {
public:
    explicit TestAStar(MapPtr map) : AStar(std::move(map)) {}
    Path trace() override {

        auto dirs = map().directions();

        auto closed = std::unordered_set<Point>();
        auto open = std::priority_queue<Path>();

        open.emplace(map().start(), map().goal());
        while (!open.empty()) {
            auto p = open.top();
            open.pop();
            auto x = p.back();
            if (closed.find(x) != closed.end())
                continue;
            if (x == map().goal())
                return p;
            closed.insert(x);
            for (auto& it: dirs)
                if (map().can_pass(x, it)) {
                    open.push(p.add(x + it, map().get_edge(x, it))); // TO-DO: cost
                }
        }
        return Path();
    }
};

class TestPrinter : public Printer {
public:
    void output(MapPtr map, const Path& path) override {
        std::cout << "+";
        for (size_t i = 0; i < map->width(); ++i) {
            std::cout << "-+";
        }
        std::cout << std::endl;

        for (size_t j = 0; j < map->height(); ++j) {
            std::cout << "|";
            for (size_t i = 0; i < map->width() - 1; ++i) {
                const char* right = map->get_edge(Point(i, j), Vector(1, 0)) == 0.f ? " " : "|";
                std::cout << " " << right;
            }
            std::cout << " |" << std::endl << "+";
            for (size_t i = 0; i < map->width() - 1; ++i) {
                const char* bottom = map->get_edge(Point(i, j), Vector(0, 1)) == 0.f ? " " : "-";
                std::cout << bottom << "+";
            }
            std::cout << " +" << std::endl;
        }
        for (auto& c : path) {
            std::cout << c.x << " " << c.y << std::endl;
        }
    }
};







int main() {

    auto map = TestMapGenerator().generate();
    TestPrinter().output(map, TestAStar(map).trace());

    auto m = maze{};
    m.generate(200, 200);
    return 0;
}

//    auto path = astar(Point(4, 0), Point(0, 4));
//    for (auto& it : path) {
//        std::cout << it.x << " " << it.y << std::endl;
//    }
//

// class TetMap {


//     float get_edge(int x1, int y1, int x2, int y2) {
//         return
//     }

// }


// struct Coord {
//     int x;
//     int y;
//     bool operator<(const Coord& o) {}
// }
//std::list<Vector> directions() {
//    std::list<Vector> dirs;
//    dirs.emplace_back(0, 1);
//    dirs.emplace_back(0, -1);
//    dirs.emplace_back(1, 0);
//    dirs.emplace_back(-1, 0);
//    return dirs;
//}



// struct PathCompator : public std::binary_function<Path, Path, bool>
// {
//     // MapPtr _map;
//     Point _goal;
//     PathCompator(Point goal) : _goal(goal) {}

//     bool operator()(const Path& lhs, const Path& rhs) const {
//         lhs.back().coordinate;
//         std::sqrtf(lhs);

//         return lhs < rhs->getTotalCost();
//     }
// };



//Path astar(Point start, Point goal) {
//
//}
