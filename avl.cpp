#include <iostream>
#include <algorithm>
#include <cassert>
#include <vector>
#include <iterator>
#include "avl.h"


int main() {
    AVLTree<int> tree;
    tree.test0(); // Тестируем дерево с 0 элементами
    tree.test1(); // Тестируем дерево с 1 элементом
    tree.test3(); // Тестируем дерево с 3 элементами
    tree.test8(); // Тестируем дерево с 8 элементами
    tree.testStd(); // Тестируем стандартные алгоритмы

    std::cout << "Preorder traversal of the constructed AVL tree is \n";
    tree.preOrder(); // Выводим дерево в порядке префиксного обхода

    return 0;
}
