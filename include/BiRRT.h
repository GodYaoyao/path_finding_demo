//
// Created by yao on 20-3-7.
//

#ifndef PATHFINDING_BI_RRT_H
#define PATHFINDING_BI_RRT_H

#include "RRT.h"

using namespace std;

class BiRRT : private RRT {
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
        vector<Node *> start_path_tree{Xs};
        vector<Node *> goal_path_tree{Xe};
        srand(time(NULL));
        for (int i = 0; i < n_iter; i++) {
            Node *random_node1 = nullptr;
            Node *random_node2 = nullptr;

            if (rand() % 100 < goal_rate) {
                random_node1 = new Node(goal.first, goal.second, nullptr);
                random_node2 = new Node(start.first, start.second, nullptr);
            } else {
                random_node1 = random_node2 = new Node(rand() % m, rand() % n, nullptr);
            }
            Node *new_node1 = calculateNewNode(grid_map, start_path_tree, random_node1);
            Node *new_node2 = calculateNewNode(grid_map, goal_path_tree, random_node2);
            if (new_node1 != nullptr)
                printLine(img, img_reso, new_node1->p, new_node1);
            if (new_node2 != nullptr)
                printLine(img, img_reso, new_node2->p, new_node2);
            delete random_node1, random_node2;

            if (calculateDistance(new_node1, new_node2) < double(step)) {
                find = true;
                break;
            }
        }

        if (find) {
            printPath(img, img_reso, start_path_tree);
            printPath(img, img_reso, goal_path_tree);
        }

        deletePath(start_path_tree);
        deletePath(goal_path_tree);

        return find;
    }
};

#endif //PATHFINDING_BI_RRT_H
