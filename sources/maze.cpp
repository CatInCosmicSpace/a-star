#include "maze.h"

#include <random>

int linear_index(int x, int y, int width) {
    return y * width + x;
};

void maze::generate(int height, int width) {
    _width = width;
    _height = height;
    map.generateWalls(height, width);

    std::map<int, std::shared_ptr<std::set<int>>> setsMap;
    for (int i = 0; i < height; ++i) {
        for (int j = 0; j < width; ++j) {
            auto index = linear_index(j, i, width);
            setsMap.insert({index, std::make_shared<std::set<int>>(std::initializer_list<int>{index})});
        }
    }

    FastNoiseLite noise;
    noise.SetNoiseType(FastNoiseLite::NoiseType_Perlin);

    auto queueHorizontal = std::vector<int>(map.horizontal.size());
    for (int i = 0; i < map.horizontal.size(); ++i) {
        queueHorizontal[i] = i;
    }
    auto queueVertical = std::vector<int>(map.vertical.size());
    for (int i = 0; i < map.vertical.size(); ++i) {
        queueVertical[i] = i;
    }
    std::shuffle(queueHorizontal.begin(), queueHorizontal.end(), std::mt19937(std::random_device()()));
    std::shuffle(queueVertical.begin(), queueVertical.end(), std::mt19937(std::random_device()()));

    while (!queueHorizontal.empty() || !queueVertical.empty()) {
        if (std::rand() % 2 == 0) { // Choose vertical
            if (queueVertical.empty()) {
                continue;
            }
            auto edgeIndex = queueVertical.back();
            queueVertical.pop_back();
            auto verticalIndex = edgeIndex % map.vertical.size();
            auto y = verticalIndex / (width - 1);
            auto x = verticalIndex % (width - 1);

            auto left = linear_index(x, y, width);
            auto right = linear_index(x + 1, y, width);

            auto& leftSet = setsMap[left];
            auto& rightSet = setsMap[right];

            if (leftSet != rightSet) {
                auto temp = rightSet;
                for (auto& it: *temp) {
                    leftSet->insert(it);
                    setsMap.erase(it);
                    setsMap.insert({it, leftSet});
                }
                map.vertical[verticalIndex] = 1.f;//(noise.GetNoise((float)x, (float)y) + 1) / 2;
            }
        } else { // Choose horizontal
            if (queueHorizontal.empty()) {
                continue;
            }
            auto edgeIndex = queueHorizontal.back();
            queueHorizontal.pop_back();
            auto horizontalIndex = edgeIndex % map.horizontal.size();
            auto y = horizontalIndex / width;
            auto x = horizontalIndex % width;

            auto up = linear_index(x, y, width);
            auto down = linear_index(x, y + 1, width);

            auto& upSet = setsMap[up];
            auto& downSet = setsMap[down];

            if (upSet != downSet) {
                auto temp = downSet;
                for (auto& it: *temp) {
                    upSet->insert(it);
                    setsMap.erase(it);
                    setsMap.insert({it, upSet});
                }
                map.horizontal[horizontalIndex] = 1.f; //(noise.GetNoise((float)x, (float)y) + 1) / 2;
            }
        }
    }
}
