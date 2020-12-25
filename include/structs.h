#pragma once
#include <list>
#include <memory>
#include <cmath>
#include <utility>


struct Point {
    int x;
    int y;
    Point() : x(0), y(0) {}
    Point(int _x, int _y) : x(_x), y(_y) {}

    bool operator ==(const Point& other) const {
        return x == other.x && y == other.y;
    }
    bool operator <(const Point& other) const {
        return y == other.y ? x < other.x : y < other.y;
    }
    Point operator +(const Point& other) const {
        return {x + other.x, y + other.y};
    }
    Point operator -(const Point& other) const {
        return {x - other.x, y - other.y};
    }  
    float length() const {
        return std::sqrt(x*x + y*y);
    }
    float euclead(const Point& to) const {
        return Point(to.x - x, to.y - y).length();
    }

    float manhattan(const Point& to) const {
        return std::fabs((float)(x - to.x)) + std::fabs((float)(y - to.y));
    }

};

namespace std
{
    template<> struct hash<Point>
    {
        std::size_t operator()(const Point& p) const noexcept
        {
            return (p.y << 16) | (p.y >> 16) ^ p.x;
        }
    };
}


typedef Point Vector;

struct PathNode {
    Point coordinate;
    float cost; // TODO: get rid of cost due to Path containing

    explicit PathNode(Point _coordinate, float _cost = 0.f) : coordinate(_coordinate), cost(_cost) {}
};


class Path : public std::list<Point> {
    float _sum {};
    float _vect {};
    Point _goal;
public:
    Path() = default;
    Path(Point start, Point goal): _sum(0.f), _vect(0.f), _goal(goal) {
        push_back(start);
    }

    Path add(Point p, float cost = 0.f) {
        Path copy(*this);
        copy.push_back(p, cost);
        return copy;
    }

    void push_back(const Point& node, float cost = 0.f)  {
        std::list<Point>::push_back(node);
        _sum += cost;
        _vect = node.manhattan(_goal); //(_goal - node).length();
    }


    float cost() const {
        return _sum;
    }
    float dist() const {
        return _vect;
    }
    float fitness() const {
        return cost() + dist();
    }

    bool operator <(const Path& other) const {
        return fitness() > other.fitness();
    }
};

class Map {
public:
    virtual size_t width() = 0;
    virtual size_t height() = 0;
    virtual Point start() = 0;
    virtual Point goal() = 0;
    virtual float get_edge(Point position, Vector direction) = 0;
    virtual std::vector<Vector> directions() = 0;
    bool can_pass(Point pos, Vector dir) {
        return get_edge(pos, dir) != 0.f; // TODO: WALL constant
    }
};
typedef std::shared_ptr<Map> MapPtr;


class MapGenerator {
public:
    virtual std::shared_ptr<Map> generate() = 0;
};
typedef std::shared_ptr<MapGenerator> MapGeneratorPtr;


class AStar {
    MapPtr _map;
public:
    explicit AStar(MapPtr map) : _map(std::move(map)) {}
    virtual Path trace() = 0;
    Map& map() {
        return *_map;
    }
};


class Printer {
    virtual void output(MapPtr map, const Path& path) = 0;
};
