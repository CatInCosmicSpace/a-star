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
#include <fstream>


std::vector<Vector> dir4 = {
        Point(1, 0),
        Point(0, 1),
        Point(-1, 0),
        Point(0, -1)
};

struct Property {
    Point point{};
    Point previous{};
    float g_score{};
    float f_score{};

    Property() = default;

    explicit Property(Point _point, Point _previous = Point(-1, -1), float _g_score = 0.f, float _f_score = 0.f)
            : point(_point), previous(_previous), g_score(_g_score), f_score(_f_score) {}

    bool operator<(const Property &other) const {
        return f_score > other.f_score;
    }
};

Path reconstruct_path(const std::unordered_map<Point, Property> &propertyMap, Point current) {
    Path total_path;
    for (auto it = propertyMap.find(current); it != propertyMap.end(); it = propertyMap.find(it->second.previous)) {
        total_path.push_front(it->second.point);
    }
    return total_path;
}

auto m = maze{};


void get_neighbors(Point point, std::vector<Point> &out) {
    out.clear();
    for (auto &dir : dir4) {
        auto new_point = point + dir;
        if (m.get_edge(point, new_point) != 0.f) {
            out.push_back(new_point);
        }
    }
}

float distance(Point a, Point b) {
    return a.manhattan(b);
}

float weight(Point a, Point b) {
    return m.get_edge(a, b);
}

Path a_star_sequential(Point start, Point goal, float (*h)(Point, Point)) {
    std::priority_queue<Property> openSet;
    openSet.emplace(start, Point(-1, -1), 0.f, h(start, goal));
    auto closedSet = std::unordered_set<Point>();


    std::unordered_map<Point, Property> propertyMap;
    propertyMap[start] = openSet.top();
    std::vector<Point> neighbors;

    while (!openSet.empty()) {
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
            auto tentative_gScore = currentProperty.g_score + weight(current, neighbor);
            auto neighborProperty = propertyMap.find(neighbor);
            if (neighborProperty == propertyMap.end() || tentative_gScore < neighborProperty->second.g_score) {
                auto property = Property(neighbor, current, tentative_gScore, tentative_gScore + h(neighbor, goal));
                openSet.push(property);
                propertyMap[neighbor] = property;
            }
        }
    }
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
    return m.get_edge(a, b);
}

// Global vars
//

std::unordered_map<Point, float> g1;
std::unordered_map<Point, float> f1;

std::unordered_map<Point, float> g2;
std::unordered_map<Point, float> f2;

struct Point_comparator {
    std::unordered_map<Point, float> &f;

    bool operator()(Point a, Point b) {
        return f[a] > f[b];
    }

    explicit Point_comparator(std::unordered_map<Point, float> &_f) : f(_f) {}
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


void a_star_new_bidirectional_kernel(
        priority_queue &open,
        std::unordered_map<Point, float> &g,
        std::unordered_map<Point, float> &f,
        std::atomic<float> &F,
        std::unordered_map<Point, Point> &o,
        float (*h)(Point),
        float (*d)(Point, Point),

        std::unordered_map<Point, float> &g_other,
        std::unordered_map<Point, float> &f_other,
        std::atomic<float> &F_other,
        std::unordered_map<Point, Point> &o_other,
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
                    if (!x_in(M, y) && (!x_in(g, y) || g[y] > (g[x] + d(x, y)))) {
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

void a_star_new_bidirectional_thread_starter(int i) {
    if (i == 0) {
        a_star_new_bidirectional_kernel(open1, g1, f1, F1, o1, h1, d1, g2, f2, F2, o2, h2);
    } else {
        a_star_new_bidirectional_kernel(open2, g2, f2, F2, o2, h2, d1, g1, f1, F1, o1, h1);
    }
}

Path a_star_new_bidirectional_parallel(Point start, Point goal) {
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

    std::thread t1(a_star_new_bidirectional_thread_starter, 0);
    std::thread t2(a_star_new_bidirectional_thread_starter, 1);
    t1.join();
    t2.join();

    Path path;
    path.push_back(middle);
    auto it = o1.find(middle);
    for (; it != o1.end(); it = o1.find(it->second)) {
        path.push_front(it->second);
    }

    for (it = o2.find(middle); it != o2.end(); it = o2.find(it->second)) {
        path.push_back(it->second);
    }

    return path;
}


void output(maze &map, Path &path) {
    std::cout << "+";
    for (size_t i = 0; i < map.width(); ++i) {
        std::cout << "-+";
    }
    std::cout << std::endl;

    for (size_t j = 0; j < map.height(); ++j) {
        std::cout << "|";
        for (size_t i = 0; i < map.width() - 1; ++i) {
            const char *right = map.get_edge(Point(i, j), Vector(i + 1, j)) == 0.f ? "|" : " ";
            const char *mid = x_find(path, Point(i, j)) == path.end() ? " " : "X";
            std::cout << mid << right;
        }
        const char *mid = x_find(path, Point(map.width() - 1, j)) == path.end() ? " " : "X";
        std::cout << mid << "|" << std::endl << "+";
        for (size_t i = 0; i < map.width(); ++i) {
            const char *bottom = map.get_edge(Point(i, j), Vector(i, j + 1)) == 0.f ? "-" : " ";
            std::cout << bottom << "+";
        }
        std::cout << std::endl;
    }
}


std::size_t measure(void (*block)(int, int), int height, int width, int count) {

    auto t1 = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < count; ++i) {
        block(height, width);
    }
    auto t2 = std::chrono::high_resolution_clock::now();

    return std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();
}

void test_parallel(int x, int y) {
    auto p = a_star_new_bidirectional_parallel(Point(0, 0), Point(x - 1, y - 1));
//    output(m, p);
//    int q = 1;
}

void test_local(int x, int y) {
    a_star_sequential(Point(0, 0), Point(x - 1, y - 1), distance);
}

#define BATCH 10
int main() {
    std::ofstream stats;
    stats.open("stats.csv");

    for (int i = 10; i < 100; ++i) {
        m.generate(i, i);
        auto totalTime = measure(test_parallel, i, i, BATCH);
        auto localTime = measure(test_local, i, i, BATCH);
        stats << i * i << "," << totalTime / BATCH << "," << localTime / BATCH << std::endl;
    }
    stats.close();

    return 0;
}
