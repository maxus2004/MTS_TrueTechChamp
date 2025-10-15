#include <algorithm>
#include <asio.hpp>
#include <asio/ip/udp.hpp>
#include <cstdint>
#include <iostream>
#include <cstdlib>
#include <chrono>
#include <locale>
#include <queue>
#include <thread>
#include <raylib.h>
#include <opencv2/opencv.hpp>
#include <utility>
#include <vector>
#include <numeric>
#include <mutex>
#include <tuple>
#include <cmath>
#include <condition_variable>
#include <atomic>
#include <map>
#include <optional>
#include <unordered_map>
#include <cstring> // for std::memcmp

#include "opencv2/core/mat.hpp"
#include "opencv2/core/types.hpp"
#include "utils.h"    // <-- contains worldToGrid declarations; DO NOT redefine them here
#include "params.h"
#include "movement.h"
#include "path.h"

using asio::ip::tcp;
using namespace std;

cv::Mat1b grid(cv::Size(GRID_W, GRID_H));          // visualization occupancy
cv::Mat1b pathfind_grid(cv::Size(GRID_W, GRID_H)); // dilated occupancy used for planning
cv::Mat1b g_grid(cv::Size(GRID_W, GRID_H));        // heatmap for display
queue<Msg> message_queue;
cv::Mat distanceGrid(GRID_H, GRID_W, CV_32F);

std::mutex path_mutex;   // protects follow_path and dstar_display_path and g_grid
std::mutex grid_mutex;   // protects pathfind_grid + grid_version
std::condition_variable grid_cv;
std::atomic<int> grid_version{0};

// show how many pathfinder updates have been produced
std::atomic<int> pathfind_updates{0};

#ifdef BACKWARDS
Robot robot{12,-1.25,0,PI};
#else
Robot robot{12,-1.25,0,0};
#endif

bool running = true;
State state = State::ManualControl;

float prev_mts_x = 0;
float prev_mts_y = 0;

// follow_path = path used by followPath; dstar_display_path = grid points used only for visualization
vector<PathPoint> follow_path;
vector<cv::Point> dstar_display_path; // in grid coordinates

bool telemetry_updated = false;

// helper: convert grid cell -> world PathPoint
PathPoint gridToWorldPoint(const cv::Point &p){
    PathPoint out;
    out.x = (p.x - GRID_W/2) * CELL_SIZE;
    out.y = (p.y - GRID_H/2) * CELL_SIZE;
    out.r = 0.0f;
    return out;
}

// Bresenham line of sight test
bool lineOfSight(const cv::Point &p1, const cv::Point &p2, const cv::Mat1b &occupancy) {
    int x0 = p1.x, y0 = p1.y;
    int x1 = p2.x, y1 = p2.y;
    int dx = abs(x1 - x0), dy = abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;

    while (true) {
        if (y0 < 0 || y0 >= occupancy.rows || x0 < 0 || x0 >= occupancy.cols)
            return false;
        if (occupancy.at<uchar>(y0, x0) != 0)
            return false;

        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 > -dy) { err -= dy; x0 += sx; }
        if (e2 < dx)  { err += dx; y0 += sy; }
    }
    return true;
}

// Simplify a raw path by removing unnecessary waypoints
vector<cv::Point> smoothPath(const vector<cv::Point> &path, const cv::Mat1b &occupancy) {
    if (path.empty()) return {};

    vector<cv::Point> result;
    result.push_back(path.front());

    int last = 0;
    for (int i = 1; i < (int)path.size(); i++) {
        if (!lineOfSight(path[last], path[i], occupancy)) {
            result.push_back(path[i - 1]);
            last = i - 1;
        }
    }
    result.push_back(path.back());
    return result;
}

void pathfinder_loop(cv::Point finish_param, cv::Point* /*start_unused*/) {
    const double INF = std::numeric_limits<double>::infinity();
    const double SQRT2 = std::sqrt(2.0);

    auto in_bounds = [&](int x, int y){
        return x >= 0 && x < GRID_W && y >= 0 && y < GRID_H;
    };

    // 8-connected neighbors (dx,dy)
    const std::vector<std::pair<int,int>> neigh8 = {
        {0,-1}, {0,1}, {-1,0}, {1,0},
        {-1,-1}, {1,-1}, {-1,1}, {1,1}
    };

    // Octile heuristic (admissible)
    auto heuristic = [&](int ax, int ay, int bx, int by)->double{
        double dx = std::abs(ax - bx);
        double dy = std::abs(ay - by);
        return (dx + dy) + (SQRT2 - 2.0) * std::min(dx, dy); // octile
    };

    struct Key { double k1, k2; };

    struct PQEntry {
        Key key;
        int x, y;
    };

    auto pq_cmp = [&](const PQEntry &a, const PQEntry &b){
        if (a.key.k1 != b.key.k1) return a.key.k1 > b.key.k1;
        return a.key.k2 > b.key.k2;
    };

    // Flat index helper
    auto idx = [&](int x, int y){ return y * GRID_W + x; };
    const int N = GRID_W * GRID_H;

    // Persistent state across iterations:
    std::vector<double> g(N, INF);
    std::vector<double> rhs(N, INF);

    double km = 0.0;
    cv::Point last_start = worldToGrid(robot.x, robot.y);

    // priority queue + map (map keyed by flat index)
    std::priority_queue<PQEntry, std::vector<PQEntry>, decltype(pq_cmp)> open(pq_cmp);
    std::unordered_map<int, Key> open_map;
    open_map.reserve(1024);

    auto calculateKey = [&](int sx, int sy, int startx, int starty, int goalx, int goaly)->Key{
        int id = idx(sx, sy);
        double val = std::min(g[id], rhs[id]);
        double k1 = val + heuristic(sx, sy, startx, starty) + km;
        double k2 = val;
        return Key{k1, k2};
    };

    auto open_insert_or_update = [&](int x, int y, const Key &k){
        int id = idx(x,y);
        open.push(PQEntry{k, x, y});
        open_map[id] = k;
    };

    auto open_remove = [&](int x, int y){
        int id = idx(x,y);
        auto it = open_map.find(id);
        if (it != open_map.end()) open_map.erase(it);
    };

    auto open_top = [&]()->std::optional<PQEntry>{
        while (!open.empty()) {
            auto ent = open.top();
            int id = idx(ent.x, ent.y);
            auto it = open_map.find(id);
            if (it == open_map.end()) {
                open.pop(); // stale
                continue;
            }
            Key curKey = it->second;
            if (std::abs(curKey.k1 - ent.key.k1) < 1e-9 && std::abs(curKey.k2 - ent.key.k2) < 1e-9) {
                return ent;
            } else {
                open.pop();
                continue;
            }
        }
        return std::nullopt;
    };

    // cost between adjacent cells (INF if blocked), corner cutting prevented
    auto edge_cost = [&](int ax, int ay, int bx, int by, const cv::Mat1b &occ)->double{
        if (!in_bounds(bx, by)) return INF;
        if (occ.at<uchar>(by, bx) != 0) return INF;
        int dx = bx - ax;
        int dy = by - ay;
        if (dx != 0 && dy != 0) {
            if (occ.at<uchar>(ay + dy, ax) != 0 || occ.at<uchar>(ay, ax + dx) != 0) return INF;
            return SQRT2;
        }
        return 1.0;
    };

    // updateVertex using flat arrays
    auto updateVertex = [&](int ux, int uy, const cv::Mat1b &occ, int startx, int starty, int goalx, int goaly){
        if (!in_bounds(ux, uy)) return;
        if (ux == goalx && uy == goaly) return; // rhs(goal) maintained externally
        int uid = idx(ux, uy);
        double minrhs = INF;
        for (auto &o : neigh8) {
            int nx = ux + o.first;
            int ny = uy + o.second;
            if (!in_bounds(nx, ny)) continue;
            double c = edge_cost(ux, uy, nx, ny, occ);
            if (!std::isfinite(c)) continue;
            double cand = c + g[idx(nx, ny)];
            if (cand < minrhs) minrhs = cand;
        }
        rhs[uid] = minrhs;
        if (std::abs(g[uid] - rhs[uid]) < 1e-9) {
            open_remove(ux, uy);
        } else {
            Key k = calculateKey(ux, uy, startx, starty, goalx, goaly);
            open_insert_or_update(ux, uy, k);
        }
    };

    // computeShortestPath (sets g; returns whether any g changed)
    auto computeShortestPath = [&](const cv::Mat1b &occ, int startx, int starty, int goalx, int goaly)->bool{
        bool any_g_changed = false;
        while (true) {
            auto topopt = open_top();
            if (!topopt.has_value()) break;
            Key k_old = topopt->key;
            int ux = topopt->x, uy = topopt->y;
            int uid = idx(ux, uy);

            Key k_start = calculateKey(startx, starty, startx, starty, goalx, goaly);
            if (k_old.k1 > k_start.k1 + 1e-9 || (std::abs(k_old.k1 - k_start.k1) < 1e-9 && k_old.k2 > k_start.k2 + 1e-9)) {
                if (std::abs(rhs[idx(startx, starty)] - g[idx(startx, starty)]) < 1e-9) break;
            }

            // pop actual top entry (lazy)
            open.pop();
            auto it = open_map.find(uid);
            if (it == open_map.end()) continue; // stale
            Key curMapKey = it->second;
            if (std::abs(curMapKey.k1 - k_old.k1) > 1e-9 || std::abs(curMapKey.k2 - k_old.k2) > 1e-9) {
                continue; // stale
            }
            open_map.erase(it);

            Key k_new = calculateKey(ux, uy, startx, starty, goalx, goaly);

            if (k_old.k1 < k_new.k1 - 1e-9 || (std::abs(k_old.k1 - k_new.k1) < 1e-9 && k_old.k2 < k_new.k2 - 1e-9)) {
                open_insert_or_update(ux, uy, k_new);
                continue;
            }

            double gval = g[uid];
            if (gval > rhs[uid] + 1e-9) {
                g[uid] = rhs[uid];
                if (std::abs(g[uid] - gval) > 1e-9) any_g_changed = true;
                // update predecessors
                for (auto &o : neigh8) {
                    int px = ux + o.first;
                    int py = uy + o.second;
                    if (!in_bounds(px, py)) continue;
                    updateVertex(px, py, occ, startx, starty, goalx, goaly);
                }
            } else {
                double g_old = gval;
                g[uid] = INF;
                if (std::abs(g_old - g[uid]) > 1e-9) any_g_changed = true;
                updateVertex(ux, uy, occ, startx, starty, goalx, goaly);
                for (auto &o : neigh8) {
                    int px = ux + o.first;
                    int py = uy + o.second;
                    if (!in_bounds(px, py)) continue;
                    updateVertex(px, py, occ, startx, starty, goalx, goaly);
                }
            }
        } // end while
        return any_g_changed;
    };

    // last occupancy snapshot (persistent)
    cv::Mat1b last_occ;
    {
        std::lock_guard<std::mutex> lk(grid_mutex);
        last_occ = pathfind_grid.clone();
    }

    // initialize goal/start tracking
    cv::Point prev_goal{-9999,-9999};
    last_start = worldToGrid(robot.x, robot.y);

    // If goal changes initially, set rhs(goal)=0 and seed open
    // We'll perform initial initialization in first iteration below.

    while (running) {
        // wait for notification or timeout
        {
            std::unique_lock<std::mutex> lk(grid_mutex);
            grid_cv.wait_for(lk, std::chrono::milliseconds(200));
        }

        // snapshot occupancy
        cv::Mat1b occ;
        {
            std::lock_guard<std::mutex> lk(grid_mutex);
            occ = pathfind_grid.clone();
        }

        // pick goal (prefer follow_path last)
        cv::Point goal = finish_param;
        {
            std::lock_guard<std::mutex> lk(path_mutex);
            if (!follow_path.empty()) goal = worldToGrid(follow_path.back());
        }
        cv::Point start = worldToGrid(robot.x, robot.y);

        if (!in_bounds(goal.x, goal.y) || !in_bounds(start.x, start.y)) {
            continue;
        }

        // Quick test: if occupancy identical, start hasn't changed cell, goal unchanged -> skip work
        bool occ_same = false;
        if (last_occ.size() == occ.size()) {
            occ_same = (std::memcmp(last_occ.data, occ.data, (size_t)occ.total() * occ.elemSize()) == 0);
        }
        bool same_cell_start = (start == last_start);
        bool same_goal = (goal == prev_goal);

        // First-time initialization when goal changed
        if (prev_goal != goal) {
            // reset g/rhs
            std::fill(g.begin(), g.end(), INF);
            std::fill(rhs.begin(), rhs.end(), INF);
            while (!open.empty()) open.pop();
            open_map.clear();
            km = 0.0;
            // rhs(goal) = 0
            rhs[idx(goal.x, goal.y)] = 0.0;
            Key kg = calculateKey(goal.x, goal.y, start.x, start.y, goal.x, goal.y);
            open_insert_or_update(goal.x, goal.y, kg);
            prev_goal = goal;
            last_occ = occ.clone();
            last_start = start;
            // initial full compute
            bool g_changed = computeShortestPath(occ, start.x, start.y, goal.x, goal.y);
            // publish even on first run
            // build heatmap & path (same as before)
            // (we'll publish below if g_changed || true)
        } else {
            // If nothing changed that affects planning, skip
            if (occ_same && same_cell_start && same_goal) {
                // nothing to do
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            // robot moved -> update km
            if (!(start == last_start)) {
                km += heuristic(last_start.x, last_start.y, start.x, start.y);
                last_start = start;
            }

            // find changed cells only if occ differs
            std::vector<cv::Point> changed_cells;
            if (!occ_same) {
                for (int y = 0; y < GRID_H; ++y) {
                    const uchar* occRow = occ.ptr<uchar>(y);
                    const uchar* lastRow = last_occ.ptr<uchar>(y);
                    for (int x = 0; x < GRID_W; ++x) {
                        if (occRow[x] != lastRow[x]) changed_cells.emplace_back(x,y);
                    }
                }
            }

            bool any_rhs_updated = false;
            if (!changed_cells.empty()) {
                for (auto &u : changed_cells) {
                    // update rhs of u and its neighbors (predecessors)
                    for (auto &o : neigh8) {
                        int px = u.x + o.first;
                        int py = u.y + o.second;
                        if (!in_bounds(px, py)) continue;
                        updateVertex(px, py, occ, start.x, start.y, goal.x, goal.y);
                        any_rhs_updated = true;
                    }
                    updateVertex(u.x, u.y, occ, start.x, start.y, goal.x, goal.y);
                    any_rhs_updated = true;
                }
                last_occ = occ.clone();
            }

            // Recompute (incrementally). computeShortestPath returns whether g changed.
            bool g_changed = computeShortestPath(occ, start.x, start.y, goal.x, goal.y);

            // If g didn't change and right-hand side updates were not performed, skip publishing
            if (!g_changed && !any_rhs_updated) {
                // nothing meaningful changed -> skip publish
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
                continue;
            }
        }

        // At this point we've run computeShortestPath (g/rhs updated)
        // Build heatmap only when g has finite entries (or changed)
        double gmin = INF, gmax = -INF;
        for (int y = 0; y < GRID_H; ++y) {
            for (int x = 0; x < GRID_W; ++x) {
                double gv = g[idx(x,y)];
                if (std::isfinite(gv)) {
                    gmin = std::min(gmin, gv);
                    gmax = std::max(gmax, gv);
                }
            }
        }
        cv::Mat1b heat(GRID_H, GRID_W);
        if (!std::isfinite(gmin) || !std::isfinite(gmax)) {
            heat.setTo(0);
        } else if (gmax == gmin) {
            for (int y = 0; y < GRID_H; ++y)
                for (int x = 0; x < GRID_W; ++x)
                    heat.at<uchar>(y,x) = std::isfinite(g[idx(x,y)]) ? 255 : 0;
        } else {
            for (int y = 0; y < GRID_H; ++y) {
                for (int x = 0; x < GRID_W; ++x) {
                    double gv = g[idx(x,y)];
                    if (!std::isfinite(gv)) {
                        heat.at<uchar>(y,x) = 0;
                    } else {
                        double norm = (gv - gmin) / (gmax - gmin);
                        uchar v = (uchar)(255.0 * (1.0 - norm)); // closer -> brighter
                        heat.at<uchar>(y,x) = v;
                    }
                }
            }
        }

        // reconstruct path from start to goal using greedy downhill on g
        std::vector<cv::Point> raw_path;
        if (std::isfinite(g[idx(start.x,start.y)])) {
            cv::Point cur = start;
            int safety = GRID_W * GRID_H;
            while (safety-- > 0) {
                raw_path.push_back(cur);
                if (cur == goal) break;

                double bestv = g[idx(cur.x,cur.y)];
                cv::Point bestp = cur;
                for (auto &o : neigh8) {
                    int nx = cur.x + o.first;
                    int ny = cur.y + o.second;
                    if (!in_bounds(nx, ny)) continue;
                    // corner cutting prevention
                    if (o.first != 0 && o.second != 0) {
                        if (occ.at<uchar>(cur.y + o.second, cur.x) != 0 || occ.at<uchar>(cur.y, cur.x + o.first) != 0) continue;
                    }
                    double ng = g[idx(nx, ny)];
                    if (!std::isfinite(ng)) continue;
                    if (ng < bestv - 1e-9) {
                        bestv = ng;
                        bestp = cv::Point(nx, ny);
                    } else if (std::abs(ng - bestv) < 1e-9) {
                        if (std::abs(o.first) == 1 && std::abs(o.second) == 1) bestp = cv::Point(nx, ny);
                    }
                }
                if (bestp == cur) break;
                cur = bestp;
            }
        }

        // smooth path
        std::vector<cv::Point> smoothed = smoothPath(raw_path, occ);

        // publish heatmap + path
        {
            std::lock_guard<std::mutex> lk(path_mutex);
            g_grid = heat;
            dstar_display_path = std::move(smoothed);
        }
        pathfind_updates.fetch_add(1, std::memory_order_relaxed);

        // sleep a tiny bit to avoid busy-looping
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    } // end while running
}


// --- rest untouched (draw_loop, main, telemetry etc.) ---

// telemetry update - unchanged
void update_telemetry(Telemetry* telemetry, tcp::socket* telemetry_socket){
    std::array<char, 1488> buf;
    asio::read(*telemetry_socket, asio::buffer(buf, 1488));
    MtsTelemetryPacket packet;
    memcpy(&packet, buf.data(), sizeof(packet));

    telemetry->gy = packet.gy;
    telemetry->ds = distance(packet.x,packet.y,prev_mts_x,prev_mts_y) * ENCODER_LINEAR_MULTIPLIER;
    telemetry->v = telemetry->ds/DT;
    memcpy(&(telemetry->distances),&(packet.distances),sizeof(telemetry->distances));
    prev_mts_x = packet.x;
    prev_mts_y = packet.y;
}

void getScanPoints(ScanPoint *points, Telemetry &telemetry, Robot &robot){
    for(int i = 0;i<360;i++){
        float a = robot.a+(45.0f-i/4.0f)/57.2958f;
        #ifdef BACKWARDS
        a += PI;
        #endif
        float d = telemetry.distances[i];
        float x = d*sin(a)+robot.x;
        float y = -d*cos(a)+robot.y;
        points[i] = {a,d,x,y};
    }
}

void start_path(queue<Msg>* messages){
    // populate the follow_path (separate from the d* display path)
    vector<PathPoint> local;
    local.push_back({12,-1.25,0});
    local.push_back({11.5,1,1});
    local.push_back({5.5,-3,1});
    local.push_back({3.5,0,1});
    local.push_back({-4,0,1});
    local.push_back({-5,-2,1});
    local.push_back({-7.7,-4,1});
    local.push_back({-7.7,0,0});

    {
        std::lock_guard<std::mutex> lock(path_mutex);
        follow_path = local; // publish follow path
    }

    // call followPath on the local vector (so followPath has stable data even if pathfinder publishes dstar path later)
    followPath(local, robot, messages);
}

thread path_thread;

void draw_loop() {
    bool path_thread_exists = false;

    InitWindow(GRID_W, GRID_H, "ArchBTW monitoring");

    Shader shader = LoadShader(NULL, "../grid_shader.fs");

    Color* pixels = new Color[GRID_W * GRID_H];
    Image img = GenImageColor(GRID_W, GRID_H, BLACK);
    img.format = PIXELFORMAT_UNCOMPRESSED_R8G8B8A8;
    Texture2D gridTex = LoadTextureFromImage(img);
    UnloadImage(img);

    cv::Mat pixelsMat(GRID_H, GRID_W, CV_8UC4, (void*)pixels);

    Image imgColor = GenImageColor(GRID_W, GRID_H, BLACK);
    imgColor.format = PIXELFORMAT_UNCOMPRESSED_R8G8B8A8;
    Texture2D colorTex = LoadTextureFromImage(imgColor);
    UnloadImage(imgColor);

    // initial distance field to some goal (visual only)
    cv::Point goal;
    {
        PathPoint t;
        t.x = -7.7;
        t.y = 0;
        goal = worldToGrid(t);
    }
    for (int y = 0; y < GRID_H; y++) {
        float dy = float(y - goal.y);
        for (int x = 0; x < GRID_W; x++) {
            float dx = float(x - goal.x);
            distanceGrid.at<float>(y, x) = sqrt(dx*dx + dy*dy);
        }
    }
    double minVal, maxVal;
    cv::minMaxLoc(distanceGrid, &minVal, &maxVal);
    distanceGrid = (distanceGrid - minVal) / (maxVal - minVal);

    cv::Mat colorGrid(GRID_H, GRID_W, CV_8UC3);
    uchar* colorData = colorGrid.data;
    for (int y = 0; y < GRID_H; y++) {
        const float* distRow = distanceGrid.ptr<float>(y);
        for (int x = 0; x < GRID_W; x++) {
            float val = distRow[x];
            int idx = (y * GRID_W + x) * 3;
            colorData[idx + 0] = (uchar)(val * 255);
            colorData[idx + 1] = 0;
            colorData[idx + 2] = (uchar)((1.0f - val) * 255);
        }
    }

    cv::Mat colorGridRGBA;
    cv::cvtColor(colorGrid, colorGridRGBA, cv::COLOR_BGR2BGRA);
    UpdateTexture(colorTex, colorGridRGBA.data);

    // pathfinder updates/s display state
    int last_pf_count = 0;
    int pf_updates_per_sec = 0;
    double last_pf_sample_time = GetTime();

    while (!WindowShouldClose()){
        if(state == State::ManualControl){
            handle_wasd();
        }
        if(IsKeyPressed(KEY_P)){
            if (!path_thread_exists) {
                path_thread = thread(start_path, &message_queue);
                path_thread_exists = true;
            }
            else {
                message_queue.push(Msg::STOPFOLOW);
            }
        }
        BeginDrawing();
        ClearBackground(RAYWHITE);

        // channel 0 <- occupancy grid
        cv::mixChannels(grid,pixelsMat,{0,0});

        // channel 1 <- g_heatmap (read under lock)
        {
            std::lock_guard<std::mutex> lock(path_mutex);
            cv::mixChannels(g_grid,pixelsMat,{0,1});
        }

        UpdateTexture(gridTex, pixels);

        // Draw textured grid with the color shader
        BeginShaderMode(shader);
        SetShaderValueTexture(shader, GetShaderLocation(shader, "uColorMap"), colorTex);
        DrawTexture(gridTex, 0, 0, WHITE);
        EndShaderMode();

        // draw follow_path (blue) -- this is the path used by followPath
        {
            std::lock_guard<std::mutex> lock(path_mutex);
            for (int i = 1; i < (int)follow_path.size(); ++i) {
                cv::Point2f p1 = worldToGrid(follow_path[i-1]);
                cv::Point2f p2 = worldToGrid(follow_path[i]);
                DrawLineEx({p1.x, p1.y}, {p2.x, p2.y}, 3, BLUE);
            }
        }

        // draw smoothed d* reconstructed path (orange)
        {
            std::lock_guard<std::mutex> lock(path_mutex);
            for (int i = 1; i < (int)dstar_display_path.size(); ++i) {
                cv::Point p1 = dstar_display_path[i-1];
                cv::Point p2 = dstar_display_path[i];
                DrawLineEx({(float)p1.x, (float)p1.y}, {(float)p2.x, (float)p2.y}, 2, ORANGE);
            }
        }

        // Draw goal marker (use follow_path's last if available)
        cv::Point goal_draw;
        {
            std::lock_guard<std::mutex> lock(path_mutex);
            if (!follow_path.empty()) {
                goal_draw = worldToGrid(follow_path.back());
            } else {
                PathPoint t; t.x = -7.7; t.y = 0; goal_draw = worldToGrid(t);
            }
        }
        DrawCircle(goal_draw.x, goal_draw.y, 5, MAGENTA);

        // draw robot
        float screen_robot_x = robot.x/CELL_SIZE+GRID_W/2;
        float screen_robot_y = robot.y/CELL_SIZE+GRID_H/2;
        float dir_x = 10*sin(robot.a);
        float dir_y = -10*cos(robot.a);
        DrawCircle(screen_robot_x, screen_robot_y, 10, GREEN);
        DrawLineEx({screen_robot_x, screen_robot_y}, {screen_robot_x+dir_x, screen_robot_y+dir_y}, 3, BLACK);

        // Sample pathfinder updates once per second
        {
            double now = GetTime();
            if (now - last_pf_sample_time >= 1.0) {
                int cur = pathfind_updates.load(std::memory_order_relaxed);
                pf_updates_per_sec = cur - last_pf_count;
                last_pf_count = cur;
                last_pf_sample_time = now;
            }
        }

        // Draw coordinates + FPS + pathfinder updates/sec
        DrawText(TextFormat("X: %.2f", robot.x), 10, 10, 20, RED);
        DrawText(TextFormat("Y: %.2f", robot.y), 10, 30, 20, RED);
        DrawText(TextFormat("%.02f FPS", 1.0/GetFrameTime()), 10, 50, 20, RED);
        DrawText(TextFormat("Pathfind/s: %d", pf_updates_per_sec), 10, 70, 20, RED);

        EndDrawing();
    }

    running = false;
    // Cleanup
    delete[] pixels;
    UnloadTexture(gridTex);
    UnloadTexture(colorTex);
    UnloadShader(shader);
    CloseWindow();
}

int main() {
    //connect to simulation
    asio::io_context io_context;
    string host = "0.0.0.0";
    string port = "5600";
    if(getenv("TEL_HOST") != NULL){
        host = getenv("TEL_HOST");
    }
    if(getenv("TEL_PORT") != NULL){
        port = getenv("TEL_PORT");
    }
    cout << "telemetry host: " << host << ":" << port << endl;
    tcp::acceptor acceptor(io_context, asio::ip::tcp::resolver(io_context).resolve(host, port).begin()->endpoint());
    tcp::socket telemetry_socket(io_context);
    acceptor.accept(telemetry_socket);

    init_movement();

    #ifdef VISUALIZATION
    thread draw_thread(draw_loop);
    #else
    path_thread = thread(start_path, &message_queue);
    #endif

    PathPoint st, ft;
    st.x = 12;
    st.y = -1.25;
    ft.x = 7.7;
    ft.y = 0;
    cv::Point s = worldToGrid(st);
    cv::Point f = worldToGrid(ft);

    // spawn the pathfinder loop (it will recompute when grid changes or periodically)
    thread dstar(pathfinder_loop, f, &s);

    Telemetry telemetry;

    // We'll notify the pathfinder whenever we update pathfind_grid in the main loop.
    // Also notify when robot moves significantly so the planner recomputes for the new start position.
    cv::Point last_robot_grid = worldToGrid(robot.x, robot.y);

    while (running) {
        //receive telemetry from robot in simulation
        update_telemetry(&telemetry, &telemetry_socket);

        //dead reckoning
        robot.a -= telemetry.gy*DT;
        robot.x += telemetry.ds*sin(robot.a);
        robot.y -= telemetry.ds*cos(robot.a);
        robot.v = telemetry.v;

        telemetry_updated = true;

        //copy grid before modifying to fix flickering in render loop
        cv::Mat1b gridCopy = grid.clone();

        //obstacle mapping
        ScanPoint scanPoints[360];
        getScanPoints(scanPoints,telemetry,robot);

        //fill white cells where there are no obstacles (fov fills)
        for(int i = 3;i<358;i++){
            cv::Point trianglePoints[3] = {
                worldToGrid(robot.x, robot.y),
                worldToGrid(scanPoints[i-1]),
                worldToGrid(scanPoints[i])
            };
            cv::fillConvexPoly(gridCopy,trianglePoints,3,2);
        }

        //fill black cells where there are obstacles
        for(int i = 1;i<360;i++){
            if(scanPoints[i].d < 8 && scanPoints[i-1].d < 8 && distance(scanPoints[i-1], scanPoints[i]) < 0.25){
                cv::line(gridCopy,worldToGrid(scanPoints[i-1]),worldToGrid(scanPoints[i]),1);
            }
        }

        //update grid for visualization
        gridCopy.copyTo(grid);

        //expand walls for pathfinding (dilate)
        cv::compare(gridCopy,1,gridCopy,cv::CMP_EQ);
        int dilation_size = 10;
        int dilation_type = cv::MORPH_ELLIPSE;
        cv::Mat element = cv::getStructuringElement( dilation_type,
                       cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                       cv::Point( dilation_size, dilation_size ) );
        cv::dilate(gridCopy,pathfind_grid,element);

        // notify pathfinder that pathfind_grid changed
        {
            std::lock_guard<std::mutex> lock(grid_mutex);
            ++grid_version;
        }
        grid_cv.notify_one();

        // also notify if robot moved to a different grid cell (so planner stops early at new start)
        cv::Point cur_robot_grid = worldToGrid(robot.x, robot.y);
        if (cur_robot_grid != last_robot_grid) {
            last_robot_grid = cur_robot_grid;
            grid_cv.notify_one();
        }
    }

    #ifdef VISUALIZATION
    draw_thread.join();
    #endif
    dstar.join();

    return 0;
}
