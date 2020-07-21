//
// Created by yao on 20-3-7.
//

#ifndef PATHFINDING_ASTAR_H
#define PATHFINDING_ASTAR_H

#include "Dijkstra.h"

using namespace std;

class Astar : private Dijkstra {
private:
    double calcH(const pair<int, int> &start, const pair<int, int> &cur, const pair<int, int> &end) {
        double dx = fabs(end.first - cur.first), dy = fabs(end.second - cur.second);
        double h1 = sqrt(pow(dx, 2) + pow(dy, 2));
        double h2 = (dx + dy) + (sqrt(2) - 2) * min(dx, dy);
        double dx_ = fabs(end.first - start.first), dy_ = fabs(end.second - start.second);
        double h3 = h1 + fabs(dx * dy_ - dx_ * dy) * 0.01;
        return h2;
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
        Node *Xs = new Node(start.first, start.second, calcH(start, start, end), nullptr);
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
                    Node *child = new Node(nx, ny, ncost + calcH(start, make_pair(nx, ny), end), cur);
                    container.push(child);
                }
            }
        }
        printPath(img, img_reso, Xe);
        deleteNode(container, visited);
        return find;
    }
};

#endif //PATHFINDING_ASTAR_H
