//
// Created by yao on 20-3-7.
//
#include "include/Astar.h"
#include "include/BiRRT.h"
#include "include/RRTStar.h"

using namespace std;

int main() {
    int x_width = 100;
    int y_width = 100;
    int img_reso = 5;

    vector<vector<bool>> grid_map(x_width, vector<bool>(y_width, false));
    for (int i = 0; i < x_width * 2 / 3; i++) {
        grid_map[i][y_width / 3 - 1] = true;
        grid_map[i][y_width / 3] = true;
        grid_map[i][y_width / 3 + 1] = true;
    }
    for (int i = x_width - 1; i > x_width / 3; i--) {
        grid_map[i][y_width * 2 / 3 - 1] = true;
        grid_map[i][y_width * 2 / 3] = true;
        grid_map[i][y_width * 2 / 3 + 1] = true;
    }

    pair<int, int> s = {x_width / 5, y_width / 6};
    pair<int, int> g = {4 * x_width / 5, 3 * y_width / 4};

    cv::namedWindow("Path Finding");
    cv::Mat back_ground(img_reso * x_width, img_reso * y_width, CV_8UC3, cv::Scalar(255, 255, 255));

    for (int i = 0; i < x_width; i++)
        for (int j = 0; j < y_width; j++)
            if (grid_map[i][j])
                cv::rectangle(back_ground,
                              cv::Point(i * img_reso, j * img_reso),
                              cv::Point((i + 1) * img_reso, (j + 1) * img_reso),
                              cv::Scalar(0, 0, 0), -1);

    try {
        std::cout << (Astar().pathFindingCore(back_ground, img_reso, grid_map, s, g) ? "True" : "False") << std::endl;
    }
    catch (string s) {
        std::cout << s << std::endl;
    }

    cv::rectangle(back_ground,
                  cv::Point(s.first * img_reso, s.second * img_reso),
                  cv::Point((s.first + 1) * img_reso, (s.second + 1) * img_reso),
                  cv::Scalar(255, 0, 0), -1);
    cv::rectangle(back_ground,
                  cv::Point(g.first * img_reso, g.second * img_reso),
                  cv::Point((g.first + 1) * img_reso, (g.second + 1) * img_reso),
                  cv::Scalar(0, 0, 255), -1);

    cv::imshow("Path Finding", back_ground);
    cv::waitKey(20 * 1000);
    return 0;
}