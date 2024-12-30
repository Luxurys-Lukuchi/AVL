#include <iostream>
#include <vector>
#include <queue>
#include <map>
#include <limits>
#include <stdexcept>
#include "Graph.h"

int main() {
    test();
    Graph<int> g(false);  // Создаем ненаправленный граф

    g.addVertex(0);
    g.addVertex(1);
    g.addVertex(2);

    g.addEdge(0, 1, 1.0);  // Добавляем ребро (0, 1) с весом 1.0
    g.addEdge(0, 1, 2.0);  // Добавляем еще одно ребро (0, 1) с весом 2.0
    g.addEdge(1, 2, 3.0);  // Добавляем ребро (1, 2) с весом 3.0

    std::vector<double> weights = g.getWeights(0, 1);
    std::cout << "Weights between 0 and 1: ";
    for (double weight : weights) {
        std::cout << weight << " ";
    }
    std::cout << std::endl;

    return 0;
}

