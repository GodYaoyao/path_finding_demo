//
// Created by yao on 20-3-7.
//

#ifndef PATHFINDING_RRT_H
#define PATHFINDING_RRT_H

#include <iostream>
#include <vector>
#include <cmath>
#include <cfloat>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

class RRT {
protected:
    int n_iter = 2000;
    int goal_rate = 5;
    int step = 2;

    int move[9][2] = {{0, 0}, {0, 1}, {1, 0}, {0, -1}, {-1, 0}, {1, 1}, {1, -1}, {-1, -1}, {-1, 1}};

    struct Node {
        double x;
        double y;
        Node *p;
        double cost;
        Node(double x = 0, double y = 0, Node *p = nullptr, double cost = 0.) : x(x), y(y), p(p), cost(cost) {}
    };

    double calculateDistance(const Node *n1, const Node *n2) {
        if (n1 == nullptr || n2 == nullptr)
            return DBL_MAX;
        return sqrt(pow(n1->x - n2->x, 2) + pow(n1->y - n2->y, 2));
    }

    bool checkCollection(const vector<vector<bool>> &grid_map, int x, int y) {
        for (int j = 0; j < 9; j++)
            if (x + move[j][0] < 0 || x + move[j][0] >= grid_map.size() || y + move[j][1] < 0
                || y + move[j][1] >= grid_map[0].size() || grid_map[x + move[j][0]][y + move[j][1]])
                return true;
        return false;
    }

    Node *calculateNewNode(const vector<vector<bool>> &grid_map,
                           vector<Node *> &path_tree,
                           Node *random_node) {
        if (random_node == nullptr)
            return nullptr;
        Node *nearest_node = nullptr;
        double min_distance = DBL_MAX;
        for (int i = 0; i < path_tree.size(); i++) {
            double temp = calculateDistance(path_tree[i], random_node);
            if (temp < min_distance) {
                min_distance = temp;
                nearest_node = path_tree[i];
            }
        }
        double new_x = nearest_node->x + step * (random_node->x - nearest_node->x) / min_distance;
        double new_y = nearest_node->y + step * (random_node->y - nearest_node->y) / min_distance;

        if (checkCollection(grid_map, int(new_x), int(new_y)))
            return nullptr;

        Node *new_node = new Node(new_x, new_y, nearest_node, nearest_node->cost + step);
        path_tree.push_back(new_node);

        return new_node;
    }

    void printLine(cv::Mat &img, const int &img_reso, const Node *n1, const Node *n2) {
        if (n1 == nullptr || n2 == nullptr)
            return;
        cv::line(img,
                 cv::Point(int(n1->x * img_reso), int(n1->y * img_reso)),
                 cv::Point(int(n2->x * img_reso), int(n2->y * img_reso)),
                 cv::Scalar(150, 150, 150), 2);
        cv::imshow("Path Finding", img);
        cv::waitKey(5);
    }

    void printPath(cv::Mat &img, const int &img_reso, vector<Node *> &path_tree) {
        Node *current_node = path_tree.back();
        while (current_node->p != nullptr) {
            cv::line(img,
                     cv::Point(int(current_node->x * img_reso), int(current_node->y * img_reso)),
                     cv::Point(int(current_node->p->x * img_reso), int(current_node->p->y * img_reso)),
                     cv::Scalar(0, 255, 0), 2);
            current_node = current_node->p;
        }
    }

    void deletePath(vector<Node *> &path_tree) {
        for (int i = 0; i < path_tree.size(); i++)
            delete path_tree[i];
    }
public:
    bool pathFindingCore(cv::Mat &img, const int &img_reso,
                         const vector<vector<bool>> &grid_map,
                         const pair<int, int> &start,
                         const pair<int, int> &goal) {
        if (grid_map.empty() || grid_map[0].empty())
            return false;
        int m = grid_map.size(), n = grid_map[0].size();
        bool find = false;
        Node *Xs = new Node(start.first, start.second, nullptr);
        Node *Xe = new Node(goal.first, goal.second, nullptr);
        vector<Node *> path_tree{Xs};
        srand(time(NULL));
        for (int i = 0; i < n_iter; i++) {
            Node *random_node = nullptr;
            if (rand() % 100 < goal_rate) {
                random_node = new Node(goal.first, goal.second, nullptr);
            } else {
                random_node = new Node(rand() % m, rand() % n, nullptr);
            }

            Node *new_node = calculateNewNode(grid_map, path_tree, random_node);
            if (new_node != nullptr)
                printLine(img, img_reso, new_node->p, new_node);
            delete random_node;

            if (calculateDistance(new_node, Xe) < step) {
                Xe->p = new_node;
                path_tree.push_back(Xe);
                find = true;
                break;
            }
        }

        if (find)
            printPath(img, img_reso, path_tree);

        deletePath(path_tree);

        return find;
    }
};

#endif //PATHFINDING_RRT_H
