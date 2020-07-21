//
// Created by yao on 20-3-7.
//

#ifndef PATHFINDING_DIJKSTRA_H
#define PATHFINDING_DIJKSTRA_H

#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <cfloat>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

class Dijkstra {
protected:
    double move[8][3] = {{0, 1, 1}, {1, 0, 1}, {0, -1, 1}, {-1, 0, 1},
                         {1, 1, sqrt(2)}, {1, -1, sqrt(2)}, {-1, -1, sqrt(2)}, {-1, 1, sqrt(2)}};
    struct Node {
        int x;
        int y;
        double cost;
        Node *p;
        Node(int x, int y, double cost, Node *p) : x(x), y(y), cost(cost), p(p) {}
    };

    struct cmp {
        bool operator()(Node *n1, Node *n2) {
            return n1->cost > n2->cost;
        }
    };

    void printPath(cv::Mat &img, const int &img_reso, Node *node) {
        while (node) {
            cv::rectangle(img,
                          cv::Point(node->x * img_reso, node->y * img_reso),
                          cv::Point((node->x + 1) * img_reso, (node->y + 1) * img_reso),
                          cv::Scalar(0, 255, 0), -1);
            node = node->p;
        }
    }

    void deleteNode(priority_queue<Node *, vector<Node *>, cmp> &container, vector<vector<Node *>> &visited) {
        while (!container.empty()) {
            delete container.top();
            container.pop();
        }
        for (int i = 0; i < visited.size(); i++)
            for (int j = 0; j < visited[0].size(); j++)
                delete visited[i][j];
    }
public:
    bool pathFindingCore(cv::Mat &img, const int &img_reso,
                         const vector<vector<bool>> &grid_map,
                         const pair<int, int> &start,
                         const pair<int, int> &end) {
        if (grid_map.empty() || grid_map[0].empty())
            return false;
        bool find = false;
        int m = grid_map.size(), n = grid_map[0].size();
        Node *Xs = new Node(start.first, start.second, 0, nullptr);
        Node *Xe = nullptr;
        priority_queue<Node *, vector<Node *>, cmp> container;
        vector<vector<Node *>> visited(m, vector<Node *>(n, nullptr));
        vector<vector<double>> path_cost(m, vector<double>(n, DBL_MAX));
        container.push(Xs);
        path_cost[Xs->x][Xs->y] = 0;
        while (!container.empty()) {
            Node *cur = container.top();
            container.pop();
            visited[cur->x][cur->y] = cur;

            cv::rectangle(img,
                          cv::Point(cur->x * img_reso, cur->y * img_reso),
                          cv::Point((cur->x + 1) * img_reso, (cur->y + 1) * img_reso),
                          cv::Scalar(150, 150, 150), -1);
            cv::imshow("Path Finding", img);
            cv::waitKey(5);

            if (cur->x == end.first && cur->y == end.second) {
                Xe = cur;
                find = true;
                break;
            }

            for (int i = 0; i < 8; i++) {
                int nx = cur->x + move[i][0];
                int ny = cur->y + move[i][1];
                double ncost = path_cost[cur->x][cur->y] + move[i][2];
                if (nx >= 0 && nx < m && ny >= 0 && ny < n && !grid_map[nx][ny] && !visited[nx][ny]
                    && ncost < path_cost[nx][ny]) {
                    path_cost[nx][ny] = ncost;
                    Node *child = new Node(nx, ny, ncost, cur);
                    container.push(child);
                }
            }
        }
        printPath(img, img_reso, Xe);
        deleteNode(container, visited);
        return find;
    }
};

#endif //PATHFINDING_DIJKSTRA_H
