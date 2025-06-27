#define _USE_MATH_DEFINES
#include <iostream>
#include <cmath>
#include <limits>
#include <cstdlib>
#include <ctime>
#include <vector>

const double MAP_WIDTH = 5.0;    // meters
const double MAP_HEIGHT = 5.0;   // meters
const double RESOLUTION = 0.01;  // 1 cm grid resolution
const int MAP_ROWS = static_cast<int>(MAP_HEIGHT / RESOLUTION);
const int MAP_COLS = static_cast<int>(MAP_WIDTH / RESOLUTION);

const double CART_SIZE = 0.1;    // meters
const double LIDAR_RANGE = 2.0;  // meters
const double ANGLE_STEP = M_PI / 180;  // 1 degree
const double ANGLE_SPAN = 2 * M_PI;    // 360 degrees

using namespace std;

struct Pose {
    double x;
    double y;
};

vector<vector<int>> createPlayground() {
    vector<vector<int>> map(MAP_ROWS, vector<int>(MAP_COLS, 0));
    // Enclose border as obstacles
    for (int i = 0; i < MAP_ROWS; ++i) {
        map[i][0] = map[i][MAP_COLS - 1] = 1;
    }
    for (int j = 0; j < MAP_COLS; ++j) {
        map[0][j] = map[MAP_ROWS - 1][j] = 1;
    }
    return map;
}

pair<int, int> worldToGrid(double x, double y) {
    int i = static_cast<int>((MAP_HEIGHT - y) / MAP_HEIGHT * (MAP_ROWS - 1));
    int j = static_cast<int>((x / MAP_WIDTH) * (MAP_COLS - 1));
    return {i, j};
}

pair<double, double> gridToWorld(int i, int j) {
    double x = (static_cast<double>(j) / (MAP_COLS - 1)) * MAP_WIDTH;
    double y = MAP_HEIGHT - ((static_cast<double>(i) / (MAP_ROWS - 1)) * MAP_HEIGHT);
    return {x, y};
}

pair<int, int> laserRange(pair<int, int> p1, pair<int, int> p2, const vector<vector<int>>& map) {
    int x0 = p1.first, y0 = p1.second;
    int x1 = p2.first, y1 = p2.second;

    int dx = abs(x1 - x0), dy = abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;

    while (true) {
        if (x0 < 0 || y0 < 0 || x0 >= MAP_ROWS || y0 >= MAP_COLS)
            return {INT_MAX, INT_MAX};
        if (map[x0][y0] == 1)
            return {x0, y0};
        if (x0 == x1 && y0 == y1)
            break;
        int e2 = 2 * err;
        if (e2 > -dy) { err -= dy; x0 += sx; }
        if (e2 < dx)  { err += dx; y0 += sy; }
    }
    return {INT_MAX, INT_MAX};
}

vector<pair<double, double>> laserScanner(double angleSpan, double angleStep, double rangeMax, const Pose& cart, const vector<vector<int>>& map) {
    vector<pair<double, double>> scan;
    pair<int, int> origin = worldToGrid(cart.x, cart.y);

    for (double angle = -angleSpan / 2; angle <= angleSpan / 2; angle += angleStep) {
        double endX = cart.x + rangeMax * cos(angle);
        double endY = cart.y + rangeMax * sin(angle);
        pair<int, int> end = worldToGrid(endX, endY);
        pair<int, int> hit = laserRange(origin, end, map);

        if (hit.first == INT_MAX || hit.second == INT_MAX) {
            scan.emplace_back(angle, numeric_limits<double>::infinity());
        } else {
            auto [hx, hy] = gridToWorld(hit.first, hit.second);
            double range = hypot(cart.x - hx, cart.y - hy);
            scan.emplace_back(angle, range);
        }
    }
    return scan;
}

Pose randomCartPose() {
    double margin = CART_SIZE / 2.0 + 0.01;
    double x = margin + (MAP_WIDTH - 2 * margin) * ((double) rand() / RAND_MAX);
    double y = margin + (MAP_HEIGHT - 2 * margin) * ((double) rand() / RAND_MAX);
    return {x, y};
}

int main() {
    srand(static_cast<unsigned>(time(0)));
    auto map = createPlayground();
    Pose cart = randomCartPose();

    cout << "Cart randomly placed at: (" << cart.x << ", " << cart.y << ")\n";

    auto scan = laserScanner(ANGLE_SPAN, ANGLE_STEP, LIDAR_RANGE, cart, map);
    for (const auto& [angle, dist] : scan) {
        cout << "Angle: " << angle << " rad, Distance: " << (isinf(dist) ? -1.0 : dist) << " m\n";
    }

    return 0;
}
