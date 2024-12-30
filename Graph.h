#pragma once
#include <iostream>
#include <vector>
#include <queue>
#include <map>
#include <limits>
#include <stdexcept>
#include <cassert>
// ���������� ���� ���-22
// ��������� ����� ��� ������������� �����
template <typename T>
class Graph {
public:
    // ����������� � ���������, �������� �� ���� ������������
    // ���������: bool isDirected - ����, �����������, �������� �� ���� ������������
    // ����������: ������
    Graph(bool isDirected = false) : directed(isDirected), vertexCount(0), edgeCount(0) {}

    // ����� ��� ���������� �������
    // ���������: T vertex - �������, ������� ����� ��������
    // ����������: ������
    void addVertex(T vertex) {
        if (adjList.find(vertex) == adjList.end()) {
            adjList[vertex] = std::vector<std::pair<T, double>>();
            vertexCount++;
        }
    }

    // ����� ��� ���������� ���� � ����
    // ���������: T u - ��������� ������� �����
    //           T v - �������� ������� �����
    //           double weight - ��� �����
    // ����������: ������
    void addEdge(T u, T v, double weight) {
        addVertex(u);  // ��������, ��� ������� u ����������
        addVertex(v);  // ��������, ��� ������� v ����������
        adjList[u].push_back(std::make_pair(v, weight));  // ��������� ����� u -> v � ����� weight
        if (!directed) {  // ���� ���� ��������������, ��������� �������� �����
            adjList[v].push_back(std::make_pair(u, weight));
        }
        edgeCount++;  // ����������� ������� ����
    }

    // ����� ����� � ������ (BFS) ������� � ������� start
    // ���������: T start - ��������� ������� ��� ������
    // ����������: std::vector<T> - ������ ������ � ������� ������
    std::vector<T> BFS(T start) const {
        if (adjList.find(start) == adjList.end()) {
            return {};  // ������� �� ����������, ���������� ������ ������
        }
        std::map<T, bool> visited;  // Map ��� ������������ ���������� ������
        std::queue<T> queue;  // ������� ��� ������ � ������
        std::vector<T> result;  // ������ ��� �������� ���������� ������

        visited[start] = true;  // �������� ��������� ������� ��� ����������
        queue.push(start);  // ��������� ��������� ������� � �������

        while (!queue.empty()) {
            T node = queue.front();  // ��������� ������� �� �������
            queue.pop();
            result.push_back(node);  // ��������� ������� � ���������

            // �������� �� ���� �������� ��������
            for (size_t i = 0; i < adjList.at(node).size(); ++i) {
                T neighbor = adjList.at(node)[i].first;
                if (!visited[neighbor]) {  // ���� �������� ������� �� ��������
                    visited[neighbor] = true;  // �������� � ��� ����������
                    queue.push(neighbor);  // ��������� � � �������
                }
            }
        }
        return result;  // ���������� ��������� ������
    }

    // ����� ����� � ������� (DFS) ������� � ������� start
    // ���������: T start - ��������� ������� ��� ������
    // ����������: std::vector<T> - ������ ������ � ������� ������
    std::vector<T> DFS(T start) const {
        if (adjList.find(start) == adjList.end()) {
            return {};  // ������� �� ����������, ���������� ������ ������
        }
        std::map<T, bool> visited;  // Map ��� ������������ ���������� ������
        std::vector<T> result;  // ������ ��� �������� ���������� ������
        DFSUtil(start, visited, result);  // ����� ��������������� ������� ��� ������
        return result;  // ���������� ��������� ������
    }

    // �������� �������� ��� ���������� ���������� �����
    // ���������: T start - ��������� ������� ��� ������ ���������� �����
    // ����������: std::map<T, double> - map, ���������� ���������� ���������� �� ��������� ������� �� ���� ���������
    std::map<T, double> dijkstra(T start) const {
        if (adjList.find(start) == adjList.end()) {
            return {};  // ������� �� ����������, ���������� ������ map
        }
        std::map<T, double> dist;  // Map ��� �������� ���������� �� ��������� �������

        // �������������� ���������� �������������� ��� ���� ������
        for (const auto& pair : adjList) {
            dist[pair.first] = std::numeric_limits<double>::infinity();
        }
        dist[start] = 0;  // ���������� �� ��������� ������� = 0

        // ������� � ����������� ��� �������� ���������� �� ������
        using NodeDist = std::pair<double, T>;
        std::priority_queue<NodeDist, std::vector<NodeDist>, std::greater<NodeDist>> pq;
        pq.push(std::make_pair(0, start));

        while (!pq.empty()) {
            double distance = pq.top().first;  // ���������� �� ������� �������
            T node = pq.top().second;
            pq.pop();

            if (distance > dist[node]) continue;  // ����������, ���� ������ ����� �������� ����

            // ������� ���� ������� ������� �������
            for (size_t i = 0; i < adjList.at(node).size(); ++i) {
                T adjNode = adjList.at(node)[i].first;
                double weight = adjList.at(node)[i].second;

                // ���� ������ ����� �������� ����, ��������� ���������� � ��������� � �������
                if (dist[node] + weight < dist[adjNode]) {
                    dist[adjNode] = dist[node] + weight;
                    pq.push(std::make_pair(dist[adjNode], adjNode));
                }
            }
        }

        return dist;  // ���������� ���������� �� ���� ������
    }

    // �������� �����-�������� ��� ���������� ���������� �����
    // ���������: T start - ��������� ������� ��� ������ ���������� �����
    // ����������: std::map<T, double> - map, ���������� ���������� ���������� �� ��������� ������� �� ���� ���������
    std::map<T, double> bellmanFord(T start) const {
        if (adjList.find(start) == adjList.end()) {
            return {};  // ������� �� ����������, ���������� ������ map
        }
        std::map<T, double> dist;  // Map ��� �������� ���������� �� ��������� �������

        // �������������� ���������� �������������� ��� ���� ������
        for (const auto& pair : adjList) {
            dist[pair.first] = std::numeric_limits<double>::infinity();
        }
        dist[start] = 0;  // ���������� �� ��������� ������� = 0

        // ��������� ������� V-1 ���, ��� V - ���������� ������
        for (size_t i = 0; i < adjList.size() - 1; ++i) {
            for (const auto& pair : adjList) {
                T u = pair.first;
                // ��������� ��� ���� u -> v
                for (size_t j = 0; j < pair.second.size(); ++j) {
                    T v = pair.second[j].first;
                    double weight = pair.second[j].second;

                    // ���� ������ ����� �������� ����, ��������� ����������
                    if (dist[u] != std::numeric_limits<double>::infinity() && dist[u] + weight < dist[v]) {
                        dist[v] = dist[u] + weight;
                    }
                }
            }
        }

        // �������� �� ������� ����� � ������������� �����
        for (const auto& pair : adjList) {
            T u = pair.first;
            for (size_t j = 0; j < pair.second.size(); ++j) {
                T v = pair.second[j].first;
                double weight = pair.second[j].second;
                if (dist[u] != std::numeric_limits<double>::infinity() && dist[u] + weight < dist[v]) {
                    throw std::runtime_error("���� �������� ���� � ������������� �����");
                }
            }
        }

        return dist;  // ���������� ���������� �� ���� ������
    }

    // ��������� ���������� ����
    // ���������: ������
    // ����������: int - ���������� ���� � �����
    int getEdgeCount() const {
        return edgeCount;
    }

    // ��������� ���������� ������
    // ���������: ������
    // ����������: int - ���������� ������ � �����
    int getVertexCount() const {
        return vertexCount;
    }

    // ��������� �������������� �����
    // ���������: ������
    // ����������: bool - true, ���� ���� ������������, ����� false
    bool isDirected() const {
        return directed;
    }

    // ��������� ���� �����
    // ���������: T u - ��������� ������� �����
    //           T v - �������� ������� �����
    // ����������: std::vector<double> - ������ ����� ����� ����� ��������� u � v
    std::vector<double> getWeights(T u, T v) const {
        std::vector<double> weights;
        for (const auto& pair : adjList.at(u)) {
            if (pair.first == v) {
                weights.push_back(pair.second);
            }
        }
        return weights;  // ���������� ���� ����� ����� u � v
    }

    // �������� �����
    // ���������: T u - ��������� ������� �����
    //           T v - �������� ������� �����
    //           double weight - ��� �����
    // ����������: ������
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

    // �������� �������
    // ���������: T v - �������, ������� ����� �������
    // ����������: ������
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

    // ��������� ��������� ���������
    // ���������: ������
    // ����������: std::vector<std::vector<T>> - ������ ��������, ��� ������ ������ ������������ ����� ���������� ���������
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
    std::map<T, std::vector<std::pair<T, double>>> adjList;  // ������ ���������

    int vertexCount;  // ����� ������
    int edgeCount;    // ����� ����
    bool directed;    // �������������� �����

    // ��������������� ������� ��� ������������ ������ � �������
    // ���������: T node - ������� ������� ��� ������
    //           std::map<T, bool>& visited - map ��� ������������ ���������� ������
    //           std::vector<T>& result - ������ ��� �������� ���������� ������
    // ����������: ������
    void DFSUtil(T node, std::map<T, bool>& visited, std::vector<T>& result) const {
        visited[node] = true;  // �������� ������� ������� ��� ����������
        result.push_back(node);  // ��������� ������� � ���������

        // �������� �� ���� �������� ��������
        for (size_t i = 0; i < adjList.at(node).size(); ++i) {
            T neighbor = adjList.at(node)[i].first;
            if (!visited[neighbor]) {
                DFSUtil(neighbor, visited, result);  // ����������� ����� ��� ������������ �������
            }
        }
    }
};


// �������� �������
void test() {
    // ������������ ���� � �������������� ������
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
    assert(comps1.size() == 1);  // ���������, ��� ���� ������� �� ����� ���������� ���������
    std::cout << "Test 1 passed" << std::endl;

    std::map<int, double> d = g.bellmanFord(0);

    assert(d[0] == 0);  // ���������, ��� ���������� �� ������� 0 �� ����� ���� ����� 0
    assert(d[1] == -1);  // ���������, ��� ���������� �� ������� 0 �� ������� 1 ����� -1
    assert(d[2] == 2);  // ���������, ��� ���������� �� ������� 0 �� ������� 2 ����� 2
    assert(d[3] == -2);  // ���������, ��� ���������� �� ������� 0 �� ������� 3 ����� -2
    assert(d[4] == 1);  // ���������, ��� ���������� �� ������� 0 �� ������� 4 ����� 1
    std::cout << "Test 2 passed" << std::endl;

    std::vector<int> s = g.BFS(0);
    std::cout << "BFS order: ";
    for (const auto& vertex : s) {
        std::cout << vertex << " ";
    }
    std::cout << std::endl;
    assert(s[0] == 0);  // ���������, ��� ������ ������� � BFS - ��� ��������� ������� 0
    assert(s[1] == 1);  // ���������, ��� ������ ������� � BFS - ��� ������� 1
    assert(s[2] == 2);  // ���������, ��� ������ ������� � BFS - ��� ������� 2
    assert(s[3] == 3);  // ���������, ��� ��������� ������� � BFS - ��� ������� 3
    assert(s[4] == 4);  // ���������, ��� ����� ������� � BFS - ��� ������� 4
    std::cout << "Test 3 passed" << std::endl;

    s = g.DFS(0);
    std::cout << "DFS order: ";
    for (const auto& vertex : s) {
        std::cout << vertex << " ";
    }
    std::cout << std::endl;
    assert(s[0] == 0);  // ���������, ��� ������ ������� � DFS - ��� ��������� ������� 0
    assert(s[1] == 1);  // ���������, ��� ������ ������� � DFS - ��� ������� 1
    assert(s[2] == 2);  // ���������, ��� ������ ������� � DFS - ��� ������� 2
    assert(s[3] == 3);  // ���������, ��� ��������� ������� � DFS - ��� ������� 3
    assert(s[4] == 4);  // ���������, ��� ����� ������� � DFS - ��� ������� 4
    std::cout << "Test 4 passed" << std::endl;

    assert(g.getWeights(0, 1).size() == 1 && g.getWeights(0, 1)[0] == -1);  // ���������, ��� ��� ����� (0, 1) ����� -1
    assert(g.getWeights(1, 4).size() == 1 && g.getWeights(1, 4)[0] == 2);  // ���������, ��� ��� ����� (1, 4) ����� 2
    std::cout << "Test 5 passed" << std::endl;

    g.deleteEdge(1, 4, 2);
    assert(g.getWeights(1, 4).size() == 0);  // ���������, ��� ����� (1, 4) ���� �������
    std::cout << "Test 6 passed" << std::endl;

    g.deleteVertex(0);

    g.addEdge(0, 3, 2);
    d = g.bellmanFord(0);

    assert(d[0] == 0);  // ���������, ��� ���������� �� ������� 0 �� ����� ���� ����� 0
    assert(d[1] == 3);  // ���������, ��� ���������� �� ������� 0 �� ������� 1 ����� 3
    std::cout << "Test 7 passed" << std::endl;

    Graph<int> one(false);
    one.addVertex(0);  // ��������� ������� 0 � ����

    d = one.bellmanFord(0);
    assert(d.size() == 1);  // ���������, ��� ���� ������� �� ����� �������
    assert(d[0] == 0);  // ���������, ��� ���������� �� ������� 0 �� ����� ���� ����� 0
    std::cout << "Test 8 passed" << std::endl;

    d = one.dijkstra(0);

    assert(d.size() == 1);  // ���������, ��� ���� ������� �� ����� �������
    assert(d[0] == 0);  // ���������, ��� ���������� �� ������� 0 �� ����� ���� ����� 0
    std::cout << "Test 9 passed" << std::endl;

    s = one.BFS(0);
    std::cout << "BFS order: ";
    for (const auto& vertex : s) {
        std::cout << vertex << " ";
    }
    std::cout << std::endl;
    assert(s.size() == 1 && s[0] == 0);  // ���������, ��� BFS ���������� ���� ������� 0
    s = one.DFS(0);
    std::cout << "DFS order: ";
    for (const auto& vertex : s) {
        std::cout << vertex << " ";
    }
    std::cout << std::endl;
    assert(s.size() == 1 && s[0] == 0);  // ���������, ��� DFS ���������� ���� ������� 0
    std::cout << "Test 10 passed" << std::endl;

    // ���� ��� ������� �����
    Graph<int> empt(false);

    s = empt.BFS(0);
    std::cout << "BFS order: ";
    for (const auto& vertex : s) {
        std::cout << vertex << " ";
    }
    std::cout << std::endl;
    assert(s.size() == 0);  // ���������, ��� BFS ���������� ������ ������ ��� ������� �����
    s = empt.DFS(0);
    std::cout << "DFS order: ";
    for (const auto& vertex : s) {
        std::cout << vertex << " ";
    }
    std::cout << std::endl;
    assert(s.size() == 0);  // ���������, ��� DFS ���������� ������ ������ ��� ������� �����
    std::cout << "Test 11 passed" << std::endl;
    // ���� ��� ����� � ����� ������������ ���������
    Graph<int> threecomp(false);

    threecomp.addVertex(0);
    threecomp.addVertex(1);
    threecomp.addVertex(2);
    threecomp.addVertex(3);
    threecomp.addVertex(4);

    threecomp.addEdge(0, 1, 1);
    threecomp.addEdge(2, 3, 1);
    std::vector<std::vector<int>> comps = threecomp.findComponents();
    assert(comps.size() == 3);  // ���������, ��� ���� ������� �� ��� ��������� ���������
    assert(comps[0][0] == 0 && comps[0][1] == 1);  // ���������, ��� ������ ���������� ������� �� ������ 0 � 1
    assert(comps[1][0] == 2 && comps[1][1] == 3);  // ���������, ��� ������ ���������� ������� �� ������ 2 � 3
    assert(comps[2][0] == 4);  // ���������, ��� ������ ���������� ������� �� ������� 4
    std::cout << "Test 12 passed" << std::endl;

    // ���� ��� ��������� �������� � ����� � ����� ������������ ���������
    std::map<int, double> dijkstraDist = threecomp.dijkstra(0);
    assert(dijkstraDist[0] == 0);  // ���������, ��� ���������� �� ������� 0 �� ����� ���� ����� 0
    assert(dijkstraDist[1] == 1);  // ���������, ��� ���������� �� ������� 0 �� ������� 1 ����� 1
    assert(dijkstraDist[2] == std::numeric_limits<double>::infinity());  // ���������, ��� ������� 2 ����������� �� ������� 0
    assert(dijkstraDist[3] == std::numeric_limits<double>::infinity());  // ���������, ��� ������� 3 ����������� �� ������� 0
    assert(dijkstraDist[4] == std::numeric_limits<double>::infinity());  // ���������, ��� ������� 4 ����������� �� ������� 0

    dijkstraDist = threecomp.dijkstra(2);
    assert(dijkstraDist[2] == 0);  // ���������, ��� ���������� �� ������� 2 �� ����� ���� ����� 0
    assert(dijkstraDist[3] == 1);  // ���������, ��� ���������� �� ������� 2 �� ������� 3 ����� 1
    assert(dijkstraDist[0] == std::numeric_limits<double>::infinity());  // ���������, ��� ������� 0 ����������� �� ������� 2
    assert(dijkstraDist[1] == std::numeric_limits<double>::infinity());  // ���������, ��� ������� 1 ����������� �� ������� 2
    assert(dijkstraDist[4] == std::numeric_limits<double>::infinity());  // ���������, ��� ������� 4 ����������� �� ������� 2

    std::cout << "Test 12.5 passed" << std::endl;
    // �������������� ���� � �������������� ������
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

    assert(d[0] == 0);  // ���������, ��� ���������� �� ������� 0 �� ����� ���� ����� 0
    assert(d[1] == 7);  // ���������, ��� ���������� �� ������� 0 �� ������� 1 ����� 7
    assert(d[2] == 9);  // ���������, ��� ���������� �� ������� 0 �� ������� 2 ����� 9
    assert(d[3] == 11);  // ���������, ��� ���������� �� ������� 0 �� ������� 3 ����� 11
    assert(d[4] == 20);  // ���������, ��� ���������� �� ������� 0 �� ������� 4 ����� 20
    assert(d[5] == 20);  // ���������, ��� ���������� �� ������� 0 �� ������� 5 ����� 20
    std::cout << "Test 13 passed" << std::endl;

    d = r.bellmanFord(0);
    assert(d[0] == 0);  // ���������, ��� ���������� �� ������� 0 �� ����� ���� ����� 0
    assert(d[1] == 7);  // ���������, ��� ���������� �� ������� 0 �� ������� 1 ����� 7
    assert(d[2] == 9);  // ���������, ��� ���������� �� ������� 0 �� ������� 2 ����� 9
    assert(d[3] == 11);  // ���������, ��� ���������� �� ������� 0 �� ������� 3 ����� 11
    assert(d[4] == 20);  // ���������, ��� ���������� �� ������� 0 �� ������� 4 ����� 20
    assert(d[5] == 20);  // ���������, ��� ���������� �� ������� 0 �� ������� 5 ����� 20
    std::cout << "Test 14 passed" << std::endl;

    s = r.BFS(0);
    std::cout << "BFS order: ";
    for (const auto& vertex : s) {
        std::cout << vertex << " ";
    }
    std::cout << std::endl;
    assert(s.size() == 6);  // ���������, ��� BFS ���������� 6 ������
    assert(s[0] == 0);  // ���������, ��� ������ ������� � BFS - ��� ��������� ������� 0
    assert(s[1] == 1);  // ���������, ��� ������ ������� � BFS - ��� ������� 1
    assert(s[2] == 2);  // ���������, ��� ������ ������� � BFS - ��� ������� 2
    assert(s[3] == 3);  // ���������, ��� ��������� ������� � BFS - ��� ������� 3
    assert(s[4] == 4);  // ���������, ��� ����� ������� � BFS - ��� ������� 4
    assert(s[5] == 5);  // ���������, ��� ������ ������� � BFS - ��� ������� 5
    std::cout << "Test 15 passed" << std::endl;

    s = r.DFS(0);
    std::cout << "DFS order: ";
    for (const auto& vertex : s) {
        std::cout << vertex << " ";
    }
    std::cout << std::endl;
    assert(s.size() == 6);  // ���������, ��� DFS ���������� 6 ������
    assert(s[0] == 0);  // ���������, ��� ������ ������� � DFS - ��� ��������� ������� 0
    assert(s[1] == 1);  // ���������, ��� ������ ������� � DFS - ��� ������� 1
    assert(s[2] == 2);  // ���������, ��� ������ ������� � DFS - ��� ������� 2
    assert(s[3] == 4);  // ���������, ��� ��������� ������� � DFS - ��� ������� 3
    assert(s[4] == 5);  // ���������, ��� ����� ������� � DFS - ��� ������� 4
    assert(s[5] == 3);  // ���������, ��� ������ ������� � DFS - ��� ������� 5
    std::cout << "Test 16 passed" << std::endl;

    // ���� ��� ��������� ��������
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

    assert(d[0] == 0);  // ���������, ��� ���������� �� ������� 0 �� ����� ���� ����� 0
    assert(d[1] == 3);  // ���������, ��� ���������� �� ������� 0 �� ������� 1 ����� 3
    assert(d[2] == 1);  // ���������, ��� ���������� �� ������� 0 �� ������� 2 ����� 1
    assert(d[3] == 3);  // ���������, ��� ���������� �� ������� 0 �� ������� 3 ����� 3
    std::cout << "Test 17 passed" << std::endl;

    std::cout << "All tests passed!" << std::endl;
}
//������������ ���� : � ������������ ����� ����������� ����� �������������� ��� ������������� ��������� ��������� 
//����� �������� ��� ���������.��������, ����� ����� �������� ����� ���� ��������� ������������� �����, 
//��������������� ����� ��� �������������.