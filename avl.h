#pragma once
#include <iostream>
#include <algorithm>
#include <cassert>
#include <vector>
#include <iterator>

template <typename T>
class AVLTree {
private:
    struct Node {
        T key; // Ключ узла
        int height; // Высота узла
        Node* left; // Левый потомок
        Node* right; // Правый потомок

        Node(T k) : key(k), height(1), left(nullptr), right(nullptr) {} // Конструктор узла
    };

    Node* root; // Корень дерева

    int height(Node* N) {
        return N ? N->height : 0; // Возвращает высоту узла или 0, если узел пустой
    }

    int getBalance(Node* N) {
        return N ? height(N->left) - height(N->right) : 0; // Возвращает баланс узла или 0, если узел пустой
    }

    Node* rightRotate(Node* y) {
        Node* x = y->left; // Левый потомок узла y
        Node* T2 = x->right; // Правый потомок узла x

        // Выполняем правый поворот
        x->right = y;
        y->left = T2;

        // Обновляем высоты
        y->height = std::max(height(y->left), height(y->right)) + 1;
        x->height = std::max(height(x->left), height(x->right)) + 1;

        // Возвращаем новый корень
        return x;
    }

    Node* leftRotate(Node* x) {
        Node* y = x->right; // Правый потомок узла x
        Node* T2 = y->left; // Левый потомок узла y

        // Выполняем левый поворот
        y->left = x;
        x->right = T2;

        // Обновляем высоты
        x->height = std::max(height(x->left), height(x->right)) + 1;
        y->height = std::max(height(y->left), height(y->right)) + 1;

        // Возвращаем новый корень
        return y;
    }

    Node* insert(Node* node, T key) {
        // 1. Выполняем стандартную вставку в BST
        if (!node) return new Node(key); // Если узел пустой, создаем новый узел

        if (key < node->key)
            node->left = insert(node->left, key); // Вставляем в левое поддерево
        else if (key > node->key)
            node->right = insert(node->right, key); // Вставляем в правое поддерево
        else
            return node; // Дублирующиеся ключи не допускаются

        // 2. Обновляем высоту текущего узла
        node->height = 1 + std::max(height(node->left), height(node->right));

        // 3. Получаем фактор баланса текущего узла, чтобы проверить, стал ли он несбалансированным
        int balance = getBalance(node);

        // Если узел стал несбалансированным, то есть 4 случая

        // Левый-Левый случай
        if (balance > 1 && key < node->left->key)
            return rightRotate(node);

        // Правый-Правый случай
        if (balance < -1 && key > node->right->key)
            return leftRotate(node);

        // Левый-Правый случай
        if (balance > 1 && key > node->left->key) {
            node->left = leftRotate(node->left);
            return rightRotate(node);
        }

        // Правый-Левый случай
        if (balance < -1 && key < node->right->key) {
            node->right = rightRotate(node->right);
            return leftRotate(node);
        }

        // Возвращаем неизменный указатель на узел
        return node;
    }

    Node* minValueNode(Node* node) {
        Node* current = node; // Начинаем с текущего узла
        while (current->left != nullptr)
            current = current->left; // Идем влево до конца
        return current; // Возвращаем узел с минимальным значением
    }

    Node* deleteNode(Node* root, T key) {
        // 1. Выполняем стандартное удаление в BST
        if (!root) return root; // Если узел пустой, возвращаем его

        if (key < root->key)
            root->left = deleteNode(root->left, key); // Удаляем из левого поддерева
        else if (key > root->key)
            root->right = deleteNode(root->right, key); // Удаляем из правого поддерева
        else {
            // Узел с одним потомком или без потомков
            if ((root->left == nullptr) || (root->right == nullptr)) {
                Node* temp = root->left ? root->left : root->right; // Получаем потомка

                // Случай без потомков
                if (temp == nullptr) {
                    temp = root;
                    root = nullptr;
                } else // Случай с одним потомком
                    *root = *temp; // Копируем содержимое непустого потомка

                delete temp; // Удаляем старый узел
            } else {
                // Узел с двумя потомками: получаем преемника (минимальный в правом поддереве)
                Node* temp = minValueNode(root->right);

                // Копируем данные преемника в текущий узел
                root->key = temp->key;

                // Удаляем преемника
                root->right = deleteNode(root->right, temp->key);
            }
        }

        // Если дерево имело только один узел, возвращаем его
        if (!root) return root;

        // 2. Обновляем высоту текущего узла
        root->height = 1 + std::max(height(root->left), height(root->right));

        // 3. Получаем фактор баланса текущего узла (чтобы проверить, стал ли он несбалансированным)
        int balance = getBalance(root);

        // Если узел стал несбалансированным, то есть 4 случая

        // Левый-Левый случай
        if (balance > 1 && getBalance(root->left) >= 0)
            return rightRotate(root);

        // Левый-Правый случай
        if (balance > 1 && getBalance(root->left) < 0) {
            root->left = leftRotate(root->left);
            return rightRotate(root);
        }

        // Правый-Правый случай
        if (balance < -1 && getBalance(root->right) <= 0)
            return leftRotate(root);

        // Правый-Левый случай
        if (balance < -1 && getBalance(root->right) > 0) {
            root->right = rightRotate(root->right);
            return leftRotate(root);
        }

        return root;
    }

    Node* updateNode(Node* node, T oldKey, T newKey) {
        if (!node) return node; // Если узел пустой, возвращаем его

        if (oldKey < node->key)
            node->left = updateNode(node->left, oldKey, newKey); // Обновляем в левом поддереве
        else if (oldKey > node->key)
            node->right = updateNode(node->right, oldKey, newKey); // Обновляем в правом поддереве
        else {
            node->key = newKey; // Обновляем ключ
            return node;
        }

        // Обновляем высоту текущего узла
        node->height = 1 + std::max(height(node->left), height(node->right));

        // Получаем фактор баланса текущего узла, чтобы проверить, стал ли он несбалансированным
        int balance = getBalance(node);

        // Если узел стал несбалансированным, то есть 4 случая

        // Левый-Левый случай
        if (balance > 1 && newKey < node->left->key)
            return rightRotate(node);

        // Правый-Правый случай
        if (balance < -1 && newKey > node->right->key)
            return leftRotate(node);

        // Левый-Правый случай
        if (balance > 1 && newKey > node->left->key) {
            node->left = leftRotate(node->left);
            return rightRotate(node);
        }

        // Правый-Левый случай
        if (balance < -1 && newKey < node->right->key) {
            node->right = rightRotate(node->right);
            return leftRotate(node);
        }

        // Возвращаем неизменный указатель на узел
        return node;
    }

    Node* searchNode(Node* node, T key) {
        if (!node || node->key == key)
            return node; // Если узел пустой или ключ найден, возвращаем узел

        if (key < node->key)
            return searchNode(node->left, key); // Ищем в левом поддереве

        return searchNode(node->right, key); // Ищем в правом поддереве
    }

    void preOrder(Node* node) {
        if (!node) return; // Если узел пустой, возвращаемся
        std::cout << node->key << " "; // Выводим ключ узла
        preOrder(node->left); // Рекурсивно выводим левое поддерево
        preOrder(node->right); // Рекурсивно выводим правое поддерево
    }

    // Функция для сбора всех ключей дерева в вектор
    void collectKeys(Node* node, std::vector<T>& keys) {
        if (!node) return; // Если узел пустой, возвращаемся
        collectKeys(node->left, keys); // Рекурсивно собираем ключи из левого поддерева
        keys.push_back(node->key); // Добавляем ключ текущего узла в вектор
        collectKeys(node->right, keys); // Рекурсивно собираем ключи из правого поддерева
    }

public:
    AVLTree() : root(nullptr) {} // Конструктор дерева

    void insert(T key) {
        root = insert(root, key); // Вставляем ключ в дерево
    }

    void remove(T key) {
        root = deleteNode(root, key); // Удаляем ключ из дерева
    }

    void update(T oldKey, T newKey) {
        root = updateNode(root, oldKey, newKey); // Обновляем ключ в дереве
    }

    Node* search(T key) {
        return searchNode(root, key); // Ищем ключ в дереве
    }

    void preOrder() {
        preOrder(root); // Выводим дерево в порядке префиксного обхода
    }

    // Функция для сбора всех ключей дерева в вектор
    std::vector<T> getKeys() {
        std::vector<T> keys;
        collectKeys(root, keys);
        return keys;
    }

    // Функция для тестирования дерева с 8 элементами
void test() {
    AVLTree<int> tree8;

    // Вставка элементов в AVL-дерево
    tree8.insert(10); // Дерево: 10
    tree8.insert(20); // Дерево: 10 (левый) - 20 (корень)
    tree8.insert(30); // Дерево: 10 (левый) - 20 (корень) - 30 (правый)
    tree8.insert(40); // Дерево: 10 (левый) - 20 (левый) - 30 (корень) - 40 (правый)
    tree8.insert(50); // Дерево: 10 (левый) - 20 (левый) - 30 (левый) - 40 (корень) - 50 (правый)
    tree8.insert(60); // Дерево: 10 (левый) - 20 (левый) - 30 (левый) - 40 (корень) - 50 (правый) - 60 (правый)
    tree8.insert(70); // Дерево: 10 (левый) - 20 (левый) - 30 (левый) - 40 (корень) - 50 (правый) - 60 (правый) - 70 (правый)
    tree8.insert(80); // Дерево: 10 (левый) - 20 (левый) - 30 (левый) - 40 (корень) - 50 (правый) - 60 (правый) - 70 (правый) - 80 (правый)

    // Ожидаемые повороты:
    // После вставки 40: правый поворот вокруг 30
    // После вставки 50: левый поворот вокруг 40
    // После вставки 60: правый поворот вокруг 50
    // После вставки 70: левый поворот вокруг 60
    // После вставки 80: правый поворот вокруг 70

    // Проверка структуры дерева
    assert(tree8.root->key == 40); // Корень
    assert(tree8.root->height == 4); // Высота корня
    assert(tree8.root->left->key == 20); // Левый потомок корня
    assert(tree8.root->right->key == 60); // Правый потомок корня
    assert(tree8.root->left->left->key == 10); // Левый потомок левого поддерева
    assert(tree8.root->left->right->key == 30); // Правый потомок левого поддерева
    assert(tree8.root->right->left->key == 50); // Левый потомок правого поддерева
    assert(tree8.root->right->right->key == 70); // Правый потомок правого поддерева
    assert(tree8.root->right->right->right->key == 80); // Правый потомок правого поддерева правого поддерева
        std::cout << "All tests passed!" << std::endl;
    }

    // Функция для тестирования дерева с 0 элементами
    void test0() {
        AVLTree<int> tree0;
        assert(tree0.root == nullptr); // Проверяем, что дерево пустое
        assert(tree0.search(10) == nullptr); // Проверяем, что элемент 10 не найден
        std::cout << "All tests for tree with 0 elements passed!" << std::endl;
    }

    // Функция для тестирования дерева с 1 элементом
    void test1() {
        AVLTree<int> tree1;
        tree1.insert(10); // Вставляем элемент 10
        assert(tree1.root->key == 10); // Проверяем, что корень дерева равен 10
        assert(tree1.root->height == 1); // Проверяем, что высота дерева равна 1
        assert(tree1.search(10)->key == 10); // Проверяем, что элемент 10 найден
        assert(tree1.search(20) == nullptr); // Проверяем, что элемент 20 не найден
        tree1.update(10, 20); // Обновляем элемент 10 на 20
        assert(tree1.root->key == 20); // Проверяем, что корень дерева равен 20
        tree1.remove(20); // Удаляем элемент 20
        assert(tree1.root == nullptr); // Проверяем, что дерево пустое
        std::cout << "All tests for tree with 1 element passed!" << std::endl;
    }

    // Функция для тестирования дерева с 3 элементами
    void test3() {
        AVLTree<int> tree3;
        tree3.insert(10); // Вставляем элемент 10
        tree3.insert(20); // Вставляем элемент 20
        tree3.insert(30); // Вставляем элемент 30
        assert(tree3.root->key == 20); // Проверяем, что корень дерева равен 20
        assert(tree3.root->height == 2); // Проверяем, что высота дерева равна 2
        assert(tree3.root->left->key == 10); // Проверяем, что левый потомок корня равен 10
        assert(tree3.root->right->key == 30); // Проверяем, что правый потомок корня равен 30
        assert(tree3.search(10)->key == 10); // Проверяем, что элемент 10 найден
        assert(tree3.search(20)->key == 20); // Проверяем, что элемент 20 найден
        assert(tree3.search(30)->key == 30); // Проверяем, что элемент 30 найден
        assert(tree3.search(40) == nullptr); // Проверяем, что элемент 40 не найден
        tree3.update(20, 25); // Обновляем элемент 20 на 25
        assert(tree3.root->key == 25); // Проверяем, что корень дерева равен 25
        tree3.remove(10); // Удаляем элемент 10
        assert(tree3.root->key == 25); // Проверяем, что корень дерева равен 25
        assert(tree3.root->left == nullptr); // Проверяем, что левый потомок корня пустой
        assert(tree3.root->right->key == 30); // Проверяем, что правый потомок корня равен 30
        std::cout << "All tests for tree with 3 elements passed!" << std::endl;
    }

    // Функция для тестирования дерева с 8 элементами
    void test8() {
        AVLTree<int> tree8;
        tree8.insert(10); // Вставляем элемент 10
        tree8.insert(20); // Вставляем элемент 20
        tree8.insert(30); // Вставляем элемент 30
        tree8.insert(40); // Вставляем элемент 40
        tree8.insert(50); // Вставляем элемент 50
        tree8.insert(60); // Вставляем элемент 60
        tree8.insert(70); // Вставляем элемент 70
        tree8.insert(80); // Вставляем элемент 80
        assert(tree8.root->key == 40); // Проверяем, что корень дерева равен 40
        assert(tree8.root->height == 4); // Проверяем, что высота дерева равна 4
        assert(tree8.root->left->key == 20); // Проверяем, что левый потомок корня равен 20
        assert(tree8.root->right->key == 60); // Проверяем, что правый потомок корня равен 60
        assert(tree8.root->left->left->key == 10); // Проверяем, что левый потомок левого потомка корня равен 10
        assert(tree8.root->left->right->key == 30); // Проверяем, что правый потомок левого потомка корня равен 30
        assert(tree8.root->right->left->key == 50); // Проверяем, что левый потомок правого потомка корня равен 50
        assert(tree8.root->right->right->key == 70); // Проверяем, что правый потомок правого потомка корня равен 70
        assert(tree8.root->right->right->right->key == 80); // Проверяем, что правый потомок правого потомка правого потомка корня равен 80
        assert(tree8.search(10)->key == 10); // Проверяем, что элемент 10 найден
        assert(tree8.search(20)->key == 20); // Проверяем, что элемент 20 найден
        assert(tree8.search(30)->key == 30); // Проверяем, что элемент 30 найден
        assert(tree8.search(40)->key == 40); // Проверяем, что элемент 40 найден
        assert(tree8.search(50)->key == 50); // Проверяем, что элемент 50 найден
        assert(tree8.search(60)->key == 60); // Проверяем, что элемент 60 найден
        assert(tree8.search(70)->key == 70); // Проверяем, что элемент 70 найден
        assert(tree8.search(80)->key == 80); // Проверяем, что элемент 80 найден
        assert(tree8.search(90) == nullptr); // Проверяем, что элемент 90 не найден
        assert(tree8.search(0) == nullptr); // Проверяем, что элемент 0 не найден
        tree8.update(20, 25); // Обновляем элемент 20 на 25
        tree8.update(60, 65); // Обновляем элемент 60 на 65
        assert(tree8.root->key == 40); // Проверяем, что корень дерева равен 40
        assert(tree8.root->left->key == 25); // Проверяем, что левый потомок корня равен 25
        assert(tree8.root->right->key == 65); // Проверяем, что правый потомок корня равен 65
        tree8.remove(10); // Удаляем элемент 10
        tree8.remove(80); // Удаляем элемент 80
        assert(tree8.root->key == 40); // Проверяем, что корень дерева равен 40
        assert(tree8.root->left->key == 25); // Проверяем, что левый потомок корня равен 25
        assert(tree8.root->right->key == 65); // Проверяем, что правый потомок корня равен 65
        assert(tree8.root->left->left == nullptr); // Проверяем, что левый потомок левого потомка корня пустой
        assert(tree8.root->right->right->right == nullptr); // Проверяем, что правый потомок правого потомка правого потомка корня пустой
        std::cout << "All tests for tree with 8 elements passed!" << std::endl;
    }

    // Функция для тестирования стандартных алгоритмов
    void testStd() {
        AVLTree<int> tree;
        tree.insert(10);
        tree.insert(20);
        tree.insert(30);
        tree.insert(40);
        tree.insert(50);
        tree.insert(60);
        tree.insert(70);
        tree.insert(80);

        // Сбор всех ключей дерева в вектор
        std::vector<int> keys = tree.getKeys();

        // Использование copy_if для копирования четных чисел
        std::vector<int> evenKeys;
        std::copy_if(keys.begin(), keys.end(), std::back_inserter(evenKeys), [](int x) { return x % 2 == 0; });
        assert(evenKeys == std::vector<int>({10, 20, 30, 40, 50, 60, 70, 80}));

        // Использование for_each для увеличения всех элементов на 1
        std::for_each(keys.begin(), keys.end(), [](int& x) { x += 1; });
        assert(keys == std::vector<int>({11, 21, 31, 41, 51, 61, 71, 81}));

        // Использование any_of для проверки наличия числа больше 50
        bool anyGreaterThan50 = std::any_of(keys.begin(), keys.end(), [](int x) { return x > 50; });
        assert(anyGreaterThan50 == true);

        // Использование all_of для проверки, что все числа больше 10
        bool allGreaterThan10 = std::all_of(keys.begin(), keys.end(), [](int x) { return x > 10; });
        assert(allGreaterThan10 == true);

        // Использование none_of для проверки, что ни одно число не меньше 10
        bool noneLessThan10 = std::none_of(keys.begin(), keys.end(), [](int x) { return x < 10; });
        assert(noneLessThan10 == true);

        // Использование transform для умножения всех элементов на 2
        std::vector<int> doubledKeys(keys.size());
        std::transform(keys.begin(), keys.end(), doubledKeys.begin(), [](int x) { return x * 2; });
        assert(doubledKeys == std::vector<int>({22, 42, 62, 82, 102, 122, 142, 162}));

        std::cout << "All standard algorithm tests passed!" << std::endl;
    }
};
