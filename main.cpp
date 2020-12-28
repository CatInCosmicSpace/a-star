#include "maze.h"
#include <iostream>
#include <unordered_set>
#include <unordered_map>
#include <queue>
#include <utility>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <functional>
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
        {1.0, 0.0, 1.0, 1.0, 1.0},
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


class ClassicAStar : public AStar {
public:
    explicit ClassicAStar(MapPtr map) : AStar(std::move(map)) {}
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

struct Property {
    Point point{};
    Point previous{};
    float g_score{};
    float f_score{};

    Property() = default;
    explicit Property(Point _point, Point _previous = Point(-1, -1), float _g_score = 0.f, float _f_score = 0.f)
            : point(_point), previous(_previous), g_score(_g_score), f_score(_f_score) {}

    bool operator <(const Property& other) const {
        return f_score > other.f_score;
    }
};

Path reconstruct_path(const std::unordered_map<Point, Property>& propertyMap, Point current)
{
    Path total_path;
//    total_path.push_front(current);
    for (auto it = propertyMap.find(current); it != propertyMap.end(); it = propertyMap.find(it->second.previous)) {
        total_path.push_front(it->second.point);
    }
    return total_path;
}

auto m = maze{};



void get_neighbors(Point point, std::vector<Point>& out) {
    out.clear();
    for (auto& dir : dir4) {
        auto new_point = point + dir;
//        if (new_point.x < 0 || new_point.y < 0 || new_point.x > 4 || new_point.y > 4)
//            continue;
//        if (data[new_point.y][new_point.x] != 0.f) {
        if (m.get_edge(point, new_point) != 0.f) {
            out.push_back(new_point);
        }
    }
}

float distance(Point a, Point b) {
    return a.manhattan(b);
}

float weight(Point a, Point b) {
    return m.get_edge(a, b); // data[b.y][b.x];
}

// A* finds a path from start to goal.
// h is the heuristic function. h(n) estimates the cost to reach goal from node n.
Path a_star_classic(Point start, Point goal, float (*h)(Point, Point)) {
    // The set of discovered nodes that may need to be (re-)expanded.
    // Initially, only the start node is known.
    // This is usually implemented as a min-heap or priority queue rather than a hash-set.
    std::priority_queue<Property> openSet;
    openSet.emplace(start, Point(-1, -1), 0.f, h(start, goal));

    auto closedSet = std::unordered_set<Point>();

    std::unordered_map<Point, Property> propertyMap;
    propertyMap[start] = openSet.top();
    // For node n, cameFrom[n] is the node immediately preceding it on the cheapest path from start
    // to n currently known.
//    auto cameFrom = std::unordered_map<Point, Point>();

    std::vector<Point> neighbors;
    // For node n, gScore[n] is the cost of the cheapest path from start to n currently known.
//    auto gScore = std::unordered_map<Point, float>(); // map with default value of Infinity
//    gScore[start] = 0;

    // For node n, fScore[n] := gScore[n] + h(n). fScore[n] represents our current best guess as to
    // how short a path from start to finish can be if it goes through n.
//    auto fScore = std::unordered_map<Point, float>(); // map with default value of Infinity
//    fScore[start] = h(start);

    while (!openSet.empty()) {
        // This operation can occur in O(1) time if openSet is a min-heap or a priority queue
        auto currentProperty = openSet.top();
        auto current = currentProperty.point;

        if (current == goal) {
            return reconstruct_path(propertyMap, current);
        }

        openSet.pop();

        if (closedSet.find(current) != closedSet.end())
            continue;
        closedSet.insert(current);

        get_neighbors(current, neighbors);
        for (auto neighbor : neighbors) {
            // d(current,neighbor) is the weight of the edge from current to neighbor
            // tentative_gScore is the distance from start to the neighbor through current
            auto tentative_gScore = currentProperty.g_score + weight(current, neighbor);
            auto neighborProperty = propertyMap.find(neighbor);
            if (neighborProperty == propertyMap.end() || tentative_gScore < neighborProperty->second.g_score) {
                auto property = Property(neighbor, current, tentative_gScore, tentative_gScore + h(neighbor, goal));
                openSet.push(property);
                propertyMap[neighbor] = property;
            }
        }
    }

    // Open set is empty but goal was never reached
    return Path();
}


#define x_find(C, V) std::find(C.begin(), C.end(), V)
#define x_with_lock(M) const std::lock_guard<std::mutex> _lock##M(M);
#define x_in(C, V) (C.find(V) != C.end())
#define INF std::numeric_limits<float>::infinity();
std::mutex L_mutex;
std::mutex M_mutex;
Point H1;
Point H2;


float h1(Point p) {
    return p.manhattan(H1);
}
float h2(Point p) {
    return p.manhattan(H2);
}
float d1(Point a, Point b) {
    return m.get_edge(a, b); //data[b.y][b.x];
}

// Global vars
//

std::unordered_map<Point, float> g1;
std::unordered_map<Point, float> f1;

std::unordered_map<Point, float> g2;
std::unordered_map<Point, float> f2;

struct Point_comparator {
    std::unordered_map<Point, float>& f;
    bool operator()(Point a, Point b) {
        return f[a] > f[b];
    }
    explicit Point_comparator(std::unordered_map<Point, float>& _f) : f(_f) {}
};
typedef std::priority_queue<Point, std::vector<Point>, Point_comparator> priority_queue;
auto open1 = priority_queue(Point_comparator(f1));
auto open2 = priority_queue(Point_comparator(f2));


std::atomic<float> F1(0.f);
std::atomic<float> F2(0.f);
// Protected by L_mutex
float L = INF;
Point middle;
// Protected by M_mutex
std::unordered_set<Point> M;
std::unordered_map<Point, Point> o1;
std::unordered_map<Point, Point> o2;


void a_star_new_bidir_kernel(
        priority_queue& open,
        std::unordered_map<Point, float>& g,
        std::unordered_map<Point, float>& f,
        std::atomic<float>& F,
        std::unordered_map<Point, Point>& o,
        float (*h)(Point),
        float (*d)(Point, Point),

        std::unordered_map<Point, float>& g_other,
        std::unordered_map<Point, float>& f_other,
        std::atomic<float>& F_other,
        std::unordered_map<Point, Point>& o_other,
        float (*h_other)(Point)) {

    std::vector<Point> neighbors;
    bool finished = false;

    while (!finished) {

        auto x = open.top();
        open.pop();
        if (M.find(x) == M.end()) {
            if (f[x] < L && g[x] + F_other - h_other(x) < L) {
                get_neighbors(x, neighbors);
                for (auto y : neighbors) {
                    // TODO: remove braces
                    if ((!x_in(M, y)) && ((!x_in(g, y)) || (g[y] > (g[x] + d(x, y))))) {
                        g[y] = g[x] + d(x, y);
                        f[y] = g[x] + h(y);
                        o[y] = x;
                        open.push(y);

                        if (x_in(g_other, y) && g[y] + g_other[y] < L) {
                            x_with_lock(L_mutex);

                            middle = y;

                            if (g[y] + g_other[y] < L)
                                L = g[y] + g_other[y];
                        }

                    }
                }
            }
            {
                x_with_lock(M_mutex);
                M.insert(x);
            }
        }
        if (!open.empty())
            F = f[open.top()];
        else
            finished = true;
    }
}

void a_star_new_bidir_thread_starter(int i) {
    if (i == 0)
        a_star_new_bidir_kernel(open1, g1, f1, F1, o1, h1, d1, g2, f2, F2, o2, h2);
    else
        a_star_new_bidir_kernel(open2, g2, f2, F2, o2, h2, d1, g1, f1, F1, o1, h1); // TODO: d2
}

Path a_star_new_bidir_parallel(Point start, Point goal) {
    open1.push(start);
    H1 = goal;
    g1[start] = 0.f;
    f1[start] = h1(start);
    F1 = f1[start];

    open2.push(goal);
    H2 = start;
    g2[goal] = 0.f;
    f2[goal] = h2(goal);
    F2 = f2[goal];

    std::thread t1(a_star_new_bidir_thread_starter, 0);
    std::thread t2(a_star_new_bidir_thread_starter, 1);
    t1.join();
    t2.join();

//    for (size_t y = 0; y < m.height(); ++y) {
//        for (size_t x = 0; x < m.width(); ++x) {
//            auto it = o1.find(Point(x, y));
//            if (it != o1.end()) {
//                std::cout << "(" << it->second.x << " " << it->second.y << ") ";
//            } else {
//                std::cout << "(- -) ";
//            }
//        }
//        std::cout << std::endl;
//    }
//    std::cout << std::endl;
//    for (size_t y = 0; y < m.height(); ++y) {
//        for (size_t x = 0; x < m.width(); ++x) {
//            auto it = o2.find(Point(x, y));
//            if (it != o2.end()) {
//                std::cout << "(" << it->second.x << " " << it->second.y << ") ";
//            } else {
//                std::cout << "(- -) ";
//            }
//        }
//        std::cout << std::endl;
//    }

    Path path;
    path.push_back(middle);
    auto it = o1.find(middle);
    for (; it != o1.end(); it = o1.find(it->second)) {
        path.push_front(it->second);
    }

    for (it = o2.find(middle); it != o2.end(); it = o2.find(it->second)) {
        path.push_back(it->second);
    }

//    for (auto& x : path) {
//        std::cout << x.x << " " << x.y << std::endl;
//    }
    return path;
}


void output(maze& map, Path& path) {
    std::cout << "+";
    for (size_t i = 0; i < map.width(); ++i) {
        std::cout << "-+";
    }
    std::cout << std::endl;

    for (size_t j = 0; j < map.height(); ++j) {
        std::cout << "|";
        for (size_t i = 0; i < map.width() - 1; ++i) {
            const char* right = map.get_edge(Point(i, j), Vector(i + 1, j)) == 0.f ? "|" : " ";
            const char* mid = x_find(path, Point(i, j)) == path.end() ? " " : "X";
            std::cout << mid << right;
        }
        const char* mid = x_find(path, Point(map.width() - 1, j)) == path.end() ? " " : "X";
        std::cout << mid << "|" << std::endl << "+";
        for (size_t i = 0; i < map.width(); ++i) {
            const char* bottom = map.get_edge(Point(i, j), Vector(i, j + 1)) == 0.f ? "-" : " ";
            std::cout << bottom << "+";
        }
        std::cout << std::endl;
    }
//    for (auto &c : path) {
//        std::cout << c.x << " " << c.y << std::endl;
//    }
}


std::size_t measure(void (*block)()) {

    auto t1 = std::chrono::high_resolution_clock::now();
    block();
    auto t2 = std::chrono::high_resolution_clock::now();

    return std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();
}

void test_parralel() {
    a_star_new_bidir_parallel(Point(0, 0), Point(49, 49));
}

void test_local() {
    a_star_classic(Point(0, 0), Point(49, 49), distance);
}



int main() {

    std::cout << "parallel" << std::endl;
    for (int i = 0; i < 10; ++i) {
        m.generate(50, 50);
        std::cout << measure(test_parralel) << std::endl;
    }

    std::cout << "local" << std::endl;
    for (int i = 0; i < 10; ++i) {
        m.generate(50, 50);

        std::cout << measure(test_local) << std::endl;
    }


//    auto p = a_star_new_bidir_parallel(Point(0, 0), Point(19, 19));
//    output(m, p);

//    auto p =
//    for (auto x : p) {
//        std::cout << x.x << " " << x.y << std::endl;
//    }
//    auto map = TestMapGenerator().generate();
//    TestPrinter().output(map, ClassicAStar(map).trace());
//
    return 0;
}
