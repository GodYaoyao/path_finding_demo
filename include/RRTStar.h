//
// Created by yao on 20-3-8.
//

#ifndef PATHFINDING_RRTSTAR_H
#define PATHFINDING_RRTSTAR_H

#include "RRT.h"

class RRTStar : private RRT {
private:

    void rewire(vector<Node *> &path_tree) {
        Node *cur = path_tree.back();
        vector<pair<Node *, double>> near_nodes;
        double dis = 50.0 * sqrt((log(path_tree.size()) / path_tree.size()));
        for (int i = 0; i < path_tree.size() - 1; i++) {
            double distance_to_near = calculateDistance(path_tree[i], cur);
            if (distance_to_near <= dis)
                near_nodes.emplace_back(make_pair(path_tree[i], distance_to_near));
        }
        for (int i = 0; i < near_nodes.size(); i++) {
            double new_cost = near_nodes[i].first->cost + near_nodes[i].second;
            if (new_cost < cur->cost) {
                cur->p = near_nodes[i].first;
                cur->cost = new_cost;
            }
        }
        //TODO: 此处更新附近节点的p和cost后，绘图上没有断开旧的
        for (int i = 0; i < near_nodes.size(); i++) {
            if (near_nodes[i].first->cost > cur->cost + near_nodes[i].second) {
                near_nodes[i].first->p = cur;
                near_nodes[i].first->cost = cur->cost + near_nodes[i].second;
            }
        }
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
        Node *Xs = new Node(start.first, start.second, nullptr, 0);
        Node *Xe = new Node(goal.first, goal.second, nullptr, DBL_MAX);
        vector<Node *> path_tree{Xs};
        srand(time(NULL));
        for (int i = 0; i < n_iter; ++i) {
            Node *random_node = nullptr;
            if (rand() % 100 < goal_rate) {
                random_node = new Node(goal.first, goal.second, nullptr);
            } else {
                random_node = new Node(rand() % m, rand() % n, nullptr);
            }

            Node *new_node = calculateNewNode(grid_map, path_tree, random_node);
            delete random_node;
            if (new_node == nullptr)
                continue;
            rewire(path_tree);
            printLine(img, img_reso, new_node, new_node->p);

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

#endif //PATHFINDING_RRTSTAR_H
