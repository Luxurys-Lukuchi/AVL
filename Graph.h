#pragma once
#include <iostream>
#include <vector>
#include <queue>
#include <map>
#include <limits>
#include <stdexcept>
#include <cassert>
// Нечталенко Илья ИВТ-22
// Шаблонный класс для представления графа
template <typename T>
class Graph {
public:
    // Конструктор с указанием, является ли граф направленным
    // Принимает: bool isDirected - флаг, указывающий, является ли граф направленным
    // Возвращает: ничего
    Graph(bool isDirected = false) : directed(isDirected), vertexCount(0), edgeCount(0) {}

    // Метод для добавления вершины
    // Принимает: T vertex - вершина, которую нужно добавить
    // Возвращает: ничего
    void addVertex(T vertex) {
        if (adjList.find(vertex) == adjList.end()) {
            adjList[vertex] = std::vector<std::pair<T, double>>();
            vertexCount++;
        }
    }

    // Метод для добавления рёбер в граф
    // Принимает: T u - начальная вершина ребра
    //           T v - конечная вершина ребра
    //           double weight - вес ребра
    // Возвращает: ничего
    void addEdge(T u, T v, double weight) {
        addVertex(u);  // Убедимся, что вершина u существует
        addVertex(v);  // Убедимся, что вершина v существует
        adjList[u].push_back(std::make_pair(v, weight));  // Добавляем ребро u -> v с весом weight
        if (!directed) {  // Если граф ненаправленный, добавляем обратное ребро
            adjList[v].push_back(std::make_pair(u, weight));
        }
        edgeCount++;  // Увеличиваем счётчик рёбер
    }

    // Обход графа в ширину (BFS) начиная с вершины start
    // Принимает: T start - начальная вершина для обхода
    // Возвращает: std::vector<T> - вектор вершин в порядке обхода
    std::vector<T> BFS(T start) const {
        if (adjList.find(start) == adjList.end()) {
            return {};  // Вершина не существует, возвращаем пустой вектор
        }
        std::map<T, bool> visited;  // Map для отслеживания посещённых вершин
        std::queue<T> queue;  // Очередь для обхода в ширину
        std::vector<T> result;  // Вектор для хранения результата обхода

        visited[start] = true;  // Отмечаем стартовую вершину как посещённую
        queue.push(start);  // Добавляем стартовую вершину в очередь

        while (!queue.empty()) {
            T node = queue.front();  // Извлекаем вершину из очереди
            queue.pop();
            result.push_back(node);  // Добавляем вершину в результат

            // Проходим по всем соседним вершинам
            for (size_t i = 0; i < adjList.at(node).size(); ++i) {
                T neighbor = adjList.at(node)[i].first;
                if (!visited[neighbor]) {  // Если соседняя вершина не посещена
                    visited[neighbor] = true;  // Отмечаем её как посещённую
                    queue.push(neighbor);  // Добавляем её в очередь
                }
            }
        }
        return result;  // Возвращаем результат обхода
    }

    // Обход графа в глубину (DFS) начиная с вершины start
    // Принимает: T start - начальная вершина для обхода
    // Возвращает: std::vector<T> - вектор вершин в порядке обхода
    std::vector<T> DFS(T start) const {
        if (adjList.find(start) == adjList.end()) {
            return {};  // Вершина не существует, возвращаем пустой вектор
        }
        std::map<T, bool> visited;  // Map для отслеживания посещённых вершин
        std::vector<T> result;  // Вектор для хранения результата обхода
        DFSUtil(start, visited, result);  // Вызов вспомогательной функции для обхода
        return result;  // Возвращаем результат обхода
    }

    // Алгоритм Дейкстры для нахождения кратчайших путей
    // Принимает: T start - начальная вершина для поиска кратчайших путей
    // Возвращает: std::map<T, double> - map, содержащий кратчайшие расстояния от начальной вершины до всех остальных
    std::map<T, double> dijkstra(T start) const {
        if (adjList.find(start) == adjList.end()) {
            return {};  // Вершина не существует, возвращаем пустой map
        }
        std::map<T, double> dist;  // Map для хранения расстояний от стартовой вершины

        // Инициализируем расстояния бесконечностью для всех вершин
        for (const auto& pair : adjList) {
            dist[pair.first] = std::numeric_limits<double>::infinity();
        }
        dist[start] = 0;  // Расстояние до стартовой вершины = 0

        // Очередь с приоритетом для хранения расстояний до вершин
        using NodeDist = std::pair<double, T>;
        std::priority_queue<NodeDist, std::vector<NodeDist>, std::greater<NodeDist>> pq;
        pq.push(std::make_pair(0, start));

        while (!pq.empty()) {
            double distance = pq.top().first;  // Расстояние до текущей вершины
            T node = pq.top().second;
            pq.pop();

            if (distance > dist[node]) continue;  // Пропускаем, если найден более короткий путь

            // Обходим всех соседей текущей вершины
            for (size_t i = 0; i < adjList.at(node).size(); ++i) {
                T adjNode = adjList.at(node)[i].first;
                double weight = adjList.at(node)[i].second;

                // Если найден более короткий путь, обновляем расстояние и добавляем в очередь
                if (dist[node] + weight < dist[adjNode]) {
                    dist[adjNode] = dist[node] + weight;
                    pq.push(std::make_pair(dist[adjNode], adjNode));
                }
            }
        }

        return dist;  // Возвращаем расстояния до всех вершин
    }

    // Алгоритм Форда-Беллмана для нахождения кратчайших путей
    // Принимает: T start - начальная вершина для поиска кратчайших путей
    // Возвращает: std::map<T, double> - map, содержащий кратчайшие расстояния от начальной вершины до всех остальных
    std::map<T, double> bellmanFord(T start) const {
        if (adjList.find(start) == adjList.end()) {
            return {};  // Вершина не существует, возвращаем пустой map
        }
        std::map<T, double> dist;  // Map для хранения расстояний от стартовой вершины

        // Инициализируем расстояния бесконечностью для всех вершин
        for (const auto& pair : adjList) {
            dist[pair.first] = std::numeric_limits<double>::infinity();
        }
        dist[start] = 0;  // Расстояние до стартовой вершины = 0

        // Повторяем процесс V-1 раз, где V - количество вершин
        for (size_t i = 0; i < adjList.size() - 1; ++i) {
            for (const auto& pair : adjList) {
                T u = pair.first;
                // Проверяем все рёбра u -> v
                for (size_t j = 0; j < pair.second.size(); ++j) {
                    T v = pair.second[j].first;
                    double weight = pair.second[j].second;

                    // Если найден более короткий путь, обновляем расстояние
                    if (dist[u] != std::numeric_limits<double>::infinity() && dist[u] + weight < dist[v]) {
                        dist[v] = dist[u] + weight;
                    }
                }
            }
        }

        // Проверка на наличие цикла с отрицательным весом
        for (const auto& pair : adjList) {
            T u = pair.first;
            for (size_t j = 0; j < pair.second.size(); ++j) {
                T v = pair.second[j].first;
                double weight = pair.second[j].second;
                if (dist[u] != std::numeric_limits<double>::infinity() && dist[u] + weight < dist[v]) {
                    throw std::runtime_error("Граф содержит цикл с отрицательным весом");
                }
            }
        }

        return dist;  // Возвращаем расстояния до всех вершин
    }

    // Получение количества рёбер
    // Принимает: ничего
    // Возвращает: int - количество рёбер в графе
    int getEdgeCount() const {
        return edgeCount;
    }

    // Получение количества вершин
    // Принимает: ничего
    // Возвращает: int - количество вершин в графе
    int getVertexCount() const {
        return vertexCount;
    }

    // Получение направленности графа
    // Принимает: ничего
    // Возвращает: bool - true, если граф направленный, иначе false
    bool isDirected() const {
        return directed;
    }

    // Получение веса ребра
    // Принимает: T u - начальная вершина ребра
    //           T v - конечная вершина ребра
    // Возвращает: std::vector<double> - вектор весов ребер между вершинами u и v
    std::vector<double> getWeights(T u, T v) const {
        std::vector<double> weights;
        for (const auto& pair : adjList.at(u)) {
            if (pair.first == v) {
                weights.push_back(pair.second);
            }
        }
        return weights;  // Возвращаем веса ребер между u и v
    }

    // Удаление ребра
    // Принимает: T u - начальная вершина ребра
    //           T v - конечная вершина ребра
    //           double weight - вес ребра
    // Возвращает: ничего
    void deleteEdge(T u, T v, double weight) {
        for (auto it = adjList[u].begin(); it != adjList[u].end(); ++it) {
            if (it->first == v && it->second == weight) {
                adjList[u].erase(it);
                break;
            }
        }
        if (!directed) {
            for (auto it = adjList[v].begin(); it != adjList[v].end(); ++it) {
                if (it->first == u && it->second == weight) {
                    adjList[v].erase(it);
                    break;
                }
            }
        }
        edgeCount--;
    }

    // Удаление вершины
    // Принимает: T v - вершина, которую нужно удалить
    // Возвращает: ничего
    void deleteVertex(T v) {
        for (auto& pair : adjList) {
            for (auto it = pair.second.begin(); it != pair.second.end(); ++it) {
                if (it->first == v) {
                    pair.second.erase(it);
                    break;
                }
            }
        }
        adjList.erase(v);
        vertexCount = adjList.size();
    }

    // Получение компонент связности
    // Принимает: ничего
    // Возвращает: std::vector<std::vector<T>> - вектор векторов, где каждый вектор представляет собой компоненту связности
    std::vector<std::vector<T>> findComponents() const {
        std::map<T, bool> visited;
        std::vector<std::vector<T>> components;
        for (const auto& pair : adjList) {
            T node = pair.first;
            if (!visited[node]) {
                std::vector<T> component;
                DFSUtil(node, visited, component);
                components.push_back(component);
            }
        }
        return components;
    }

private:
    std::map<T, std::vector<std::pair<T, double>>> adjList;  // Список смежности

    int vertexCount;  // Число вершин
    int edgeCount;    // Число рёбер
    bool directed;    // Направленность графа

    // Вспомогательная функция для рекурсивного обхода в глубину
    // Принимает: T node - текущая вершина для обхода
    //           std::map<T, bool>& visited - map для отслеживания посещённых вершин
    //           std::vector<T>& result - вектор для хранения результата обхода
    // Возвращает: ничего
    void DFSUtil(T node, std::map<T, bool>& visited, std::vector<T>& result) const {
        visited[node] = true;  // Отмечаем текущую вершину как посещённую
        result.push_back(node);  // Добавляем вершину в результат

        // Проходим по всем соседним вершинам
        for (size_t i = 0; i < adjList.at(node).size(); ++i) {
            T neighbor = adjList.at(node)[i].first;
            if (!visited[neighbor]) {
                DFSUtil(neighbor, visited, result);  // Рекурсивный вызов для непосещённой вершины
            }
        }
    }
};


// Тестовая функция
void test() {
    // Направленный граф с отрицательными весами
    Graph<int> g(true);

    g.addVertex(0);
    g.addVertex(1);
    g.addVertex(2);
    g.addVertex(3);
    g.addVertex(4);

    g.addEdge(0, 1, -1);
    g.addEdge(0, 2, 4);
    g.addEdge(1, 2, 3);
    g.addEdge(1, 3, 2);
    g.addEdge(1, 4, 2);
    g.addEdge(3, 2, 5);
    g.addEdge(3, 1, 1);
    g.addEdge(4, 3, -3);

    std::vector<std::vector<int>> comps1 = g.findComponents();
    assert(comps1.size() == 1);  // Проверяем, что граф состоит из одной компоненты связности
    std::cout << "Test 1 passed" << std::endl;

    std::map<int, double> d = g.bellmanFord(0);

    assert(d[0] == 0);  // Проверяем, что расстояние от вершины 0 до самой себя равно 0
    assert(d[1] == -1);  // Проверяем, что расстояние от вершины 0 до вершины 1 равно -1
    assert(d[2] == 2);  // Проверяем, что расстояние от вершины 0 до вершины 2 равно 2
    assert(d[3] == -2);  // Проверяем, что расстояние от вершины 0 до вершины 3 равно -2
    assert(d[4] == 1);  // Проверяем, что расстояние от вершины 0 до вершины 4 равно 1
    std::cout << "Test 2 passed" << std::endl;

    std::vector<int> s = g.BFS(0);
    std::cout << "BFS order: ";
    for (const auto& vertex : s) {
        std::cout << vertex << " ";
    }
    std::cout << std::endl;
    assert(s[0] == 0);  // Проверяем, что первая вершина в BFS - это стартовая вершина 0
    assert(s[1] == 1);  // Проверяем, что вторая вершина в BFS - это вершина 1
    assert(s[2] == 2);  // Проверяем, что третья вершина в BFS - это вершина 2
    assert(s[3] == 3);  // Проверяем, что четвертая вершина в BFS - это вершина 3
    assert(s[4] == 4);  // Проверяем, что пятая вершина в BFS - это вершина 4
    std::cout << "Test 3 passed" << std::endl;

    s = g.DFS(0);
    std::cout << "DFS order: ";
    for (const auto& vertex : s) {
        std::cout << vertex << " ";
    }
    std::cout << std::endl;
    assert(s[0] == 0);  // Проверяем, что первая вершина в DFS - это стартовая вершина 0
    assert(s[1] == 1);  // Проверяем, что вторая вершина в DFS - это вершина 1
    assert(s[2] == 2);  // Проверяем, что третья вершина в DFS - это вершина 2
    assert(s[3] == 3);  // Проверяем, что четвертая вершина в DFS - это вершина 3
    assert(s[4] == 4);  // Проверяем, что пятая вершина в DFS - это вершина 4
    std::cout << "Test 4 passed" << std::endl;

    assert(g.getWeights(0, 1).size() == 1 && g.getWeights(0, 1)[0] == -1);  // Проверяем, что вес ребра (0, 1) равен -1
    assert(g.getWeights(1, 4).size() == 1 && g.getWeights(1, 4)[0] == 2);  // Проверяем, что вес ребра (1, 4) равен 2
    std::cout << "Test 5 passed" << std::endl;

    g.deleteEdge(1, 4, 2);
    assert(g.getWeights(1, 4).size() == 0);  // Проверяем, что ребро (1, 4) было удалено
    std::cout << "Test 6 passed" << std::endl;

    g.deleteVertex(0);

    g.addEdge(0, 3, 2);
    d = g.bellmanFord(0);

    assert(d[0] == 0);  // Проверяем, что расстояние от вершины 0 до самой себя равно 0
    assert(d[1] == 3);  // Проверяем, что расстояние от вершины 0 до вершины 1 равно 3
    std::cout << "Test 7 passed" << std::endl;

    Graph<int> one(false);
    one.addVertex(0);  // Добавляем вершину 0 в граф

    d = one.bellmanFord(0);
    assert(d.size() == 1);  // Проверяем, что граф состоит из одной вершины
    assert(d[0] == 0);  // Проверяем, что расстояние от вершины 0 до самой себя равно 0
    std::cout << "Test 8 passed" << std::endl;

    d = one.dijkstra(0);

    assert(d.size() == 1);  // Проверяем, что граф состоит из одной вершины
    assert(d[0] == 0);  // Проверяем, что расстояние от вершины 0 до самой себя равно 0
    std::cout << "Test 9 passed" << std::endl;

    s = one.BFS(0);
    std::cout << "BFS order: ";
    for (const auto& vertex : s) {
        std::cout << vertex << " ";
    }
    std::cout << std::endl;
    assert(s.size() == 1 && s[0] == 0);  // Проверяем, что BFS возвращает одну вершину 0
    s = one.DFS(0);
    std::cout << "DFS order: ";
    for (const auto& vertex : s) {
        std::cout << vertex << " ";
    }
    std::cout << std::endl;
    assert(s.size() == 1 && s[0] == 0);  // Проверяем, что DFS возвращает одну вершину 0
    std::cout << "Test 10 passed" << std::endl;

    // Тест для пустого графа
    Graph<int> empt(false);

    s = empt.BFS(0);
    std::cout << "BFS order: ";
    for (const auto& vertex : s) {
        std::cout << vertex << " ";
    }
    std::cout << std::endl;
    assert(s.size() == 0);  // Проверяем, что BFS возвращает пустой вектор для пустого графа
    s = empt.DFS(0);
    std::cout << "DFS order: ";
    for (const auto& vertex : s) {
        std::cout << vertex << " ";
    }
    std::cout << std::endl;
    assert(s.size() == 0);  // Проверяем, что DFS возвращает пустой вектор для пустого графа
    std::cout << "Test 11 passed" << std::endl;
    // Тест для графа с двумя компонентами связности
    Graph<int> threecomp(false);

    threecomp.addVertex(0);
    threecomp.addVertex(1);
    threecomp.addVertex(2);
    threecomp.addVertex(3);
    threecomp.addVertex(4);

    threecomp.addEdge(0, 1, 1);
    threecomp.addEdge(2, 3, 1);
    std::vector<std::vector<int>> comps = threecomp.findComponents();
    assert(comps.size() == 3);  // Проверяем, что граф состоит из трёх компонент связности
    assert(comps[0][0] == 0 && comps[0][1] == 1);  // Проверяем, что первая компонента состоит из вершин 0 и 1
    assert(comps[1][0] == 2 && comps[1][1] == 3);  // Проверяем, что вторая компонента состоит из вершин 2 и 3
    assert(comps[2][0] == 4);  // Проверяем, что третья компонента состоит из вершины 4
    std::cout << "Test 12 passed" << std::endl;

    // Тест для алгоритма Дейкстры в графе с двумя компонентами связности
    std::map<int, double> dijkstraDist = threecomp.dijkstra(0);
    assert(dijkstraDist[0] == 0);  // Проверяем, что расстояние от вершины 0 до самой себя равно 0
    assert(dijkstraDist[1] == 1);  // Проверяем, что расстояние от вершины 0 до вершины 1 равно 1
    assert(dijkstraDist[2] == std::numeric_limits<double>::infinity());  // Проверяем, что вершина 2 недостижима из вершины 0
    assert(dijkstraDist[3] == std::numeric_limits<double>::infinity());  // Проверяем, что вершина 3 недостижима из вершины 0
    assert(dijkstraDist[4] == std::numeric_limits<double>::infinity());  // Проверяем, что вершина 4 недостижима из вершины 0

    dijkstraDist = threecomp.dijkstra(2);
    assert(dijkstraDist[2] == 0);  // Проверяем, что расстояние от вершины 2 до самой себя равно 0
    assert(dijkstraDist[3] == 1);  // Проверяем, что расстояние от вершины 2 до вершины 3 равно 1
    assert(dijkstraDist[0] == std::numeric_limits<double>::infinity());  // Проверяем, что вершина 0 недостижима из вершины 2
    assert(dijkstraDist[1] == std::numeric_limits<double>::infinity());  // Проверяем, что вершина 1 недостижима из вершины 2
    assert(dijkstraDist[4] == std::numeric_limits<double>::infinity());  // Проверяем, что вершина 4 недостижима из вершины 2

    std::cout << "Test 12.5 passed" << std::endl;
    // Ненаправленный граф с положительными весами
    Graph<int> r(false);

    r.addVertex(0);
    r.addVertex(1);
    r.addVertex(2);
    r.addVertex(3);
    r.addVertex(4);
    r.addVertex(5);

    r.addEdge(0, 1, 7);
    r.addEdge(0, 2, 9);
    r.addEdge(0, 3, 14);
    r.addEdge(1, 2, 10);
    r.addEdge(1, 4, 15);
    r.addEdge(2, 4, 11);
    r.addEdge(2, 3, 2);
    r.addEdge(3, 5, 9);
    r.addEdge(4, 5, 6);

    d = r.dijkstra(0);

    assert(d[0] == 0);  // Проверяем, что расстояние от вершины 0 до самой себя равно 0
    assert(d[1] == 7);  // Проверяем, что расстояние от вершины 0 до вершины 1 равно 7
    assert(d[2] == 9);  // Проверяем, что расстояние от вершины 0 до вершины 2 равно 9
    assert(d[3] == 11);  // Проверяем, что расстояние от вершины 0 до вершины 3 равно 11
    assert(d[4] == 20);  // Проверяем, что расстояние от вершины 0 до вершины 4 равно 20
    assert(d[5] == 20);  // Проверяем, что расстояние от вершины 0 до вершины 5 равно 20
    std::cout << "Test 13 passed" << std::endl;

    d = r.bellmanFord(0);
    assert(d[0] == 0);  // Проверяем, что расстояние от вершины 0 до самой себя равно 0
    assert(d[1] == 7);  // Проверяем, что расстояние от вершины 0 до вершины 1 равно 7
    assert(d[2] == 9);  // Проверяем, что расстояние от вершины 0 до вершины 2 равно 9
    assert(d[3] == 11);  // Проверяем, что расстояние от вершины 0 до вершины 3 равно 11
    assert(d[4] == 20);  // Проверяем, что расстояние от вершины 0 до вершины 4 равно 20
    assert(d[5] == 20);  // Проверяем, что расстояние от вершины 0 до вершины 5 равно 20
    std::cout << "Test 14 passed" << std::endl;

    s = r.BFS(0);
    std::cout << "BFS order: ";
    for (const auto& vertex : s) {
        std::cout << vertex << " ";
    }
    std::cout << std::endl;
    assert(s.size() == 6);  // Проверяем, что BFS возвращает 6 вершин
    assert(s[0] == 0);  // Проверяем, что первая вершина в BFS - это стартовая вершина 0
    assert(s[1] == 1);  // Проверяем, что вторая вершина в BFS - это вершина 1
    assert(s[2] == 2);  // Проверяем, что третья вершина в BFS - это вершина 2
    assert(s[3] == 3);  // Проверяем, что четвертая вершина в BFS - это вершина 3
    assert(s[4] == 4);  // Проверяем, что пятая вершина в BFS - это вершина 4
    assert(s[5] == 5);  // Проверяем, что шестая вершина в BFS - это вершина 5
    std::cout << "Test 15 passed" << std::endl;

    s = r.DFS(0);
    std::cout << "DFS order: ";
    for (const auto& vertex : s) {
        std::cout << vertex << " ";
    }
    std::cout << std::endl;
    assert(s.size() == 6);  // Проверяем, что DFS возвращает 6 вершин
    assert(s[0] == 0);  // Проверяем, что первая вершина в DFS - это стартовая вершина 0
    assert(s[1] == 1);  // Проверяем, что вторая вершина в DFS - это вершина 1
    assert(s[2] == 2);  // Проверяем, что третья вершина в DFS - это вершина 2
    assert(s[3] == 4);  // Проверяем, что четвертая вершина в DFS - это вершина 3
    assert(s[4] == 5);  // Проверяем, что пятая вершина в DFS - это вершина 4
    assert(s[5] == 3);  // Проверяем, что шестая вершина в DFS - это вершина 5
    std::cout << "Test 16 passed" << std::endl;

    // Тест для алгоритма Дейкстры
    Graph<int> dijkstraGraph(false);

    dijkstraGraph.addVertex(0);
    dijkstraGraph.addVertex(1);
    dijkstraGraph.addVertex(2);
    dijkstraGraph.addVertex(3);

    dijkstraGraph.addEdge(0, 1, 4);
    dijkstraGraph.addEdge(0, 2, 1);
    dijkstraGraph.addEdge(1, 2, 2);
    dijkstraGraph.addEdge(1, 3, 5);
    dijkstraGraph.addEdge(2, 3, 2);

    d = dijkstraGraph.dijkstra(0);

    assert(d[0] == 0);  // Проверяем, что расстояние от вершины 0 до самой себя равно 0
    assert(d[1] == 3);  // Проверяем, что расстояние от вершины 0 до вершины 1 равно 3
    assert(d[2] == 1);  // Проверяем, что расстояние от вершины 0 до вершины 2 равно 1
    assert(d[3] == 3);  // Проверяем, что расстояние от вершины 0 до вершины 3 равно 3
    std::cout << "Test 17 passed" << std::endl;

    std::cout << "All tests passed!" << std::endl;
}
//Транспортные сети : В транспортных сетях мультиграфы могут использоваться для моделирования различных маршрутов 
//между городами или станциями.Например, между двумя городами может быть несколько автомобильных дорог, 
//железнодорожных линий или авиамаршрутов.