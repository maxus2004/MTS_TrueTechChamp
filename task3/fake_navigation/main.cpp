include <asio.hpp>
#include <asio/ip/udp.hpp>
#include <asio/ip/tcp.hpp>

#include <raylib.h>
#include <opencv2/opencv.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <mutex>
#include <queue>
#include <thread>
#include <unordered_map>
#include <vector>

#include "utils.h"
#include "params.h"
#include "movement.h"
#include "path.h"

using asio::ip::tcp;
using asio::ip::udp;
using asio::error_code;
using namespace std::chrono_literals;
using namespace std;


static const float LIDAR_ROT_DEG = [](){
    const char* v = getenv("LIDAR_ROT_DEG");
    if(!v) return 90.0f;
    try { return std::stof(v); } catch(...) { return 90.0f; }
}();
static const float LIDAR_ROT_RAD = LIDAR_ROT_DEG * 3.14159265358979323846f / 180.0f;

cv::Mat1b grid(cv::Size(GRID_W, GRID_H));
cv::Mat1b pathfind_grid(cv::Size(GRID_W, GRID_H));
queue<Msg> message_queue;

#ifdef BACKWARDS
Robot robot{12, -1.25, 0, PI};
#else
Robot robot{12, -1.25, 0, 0};
#endif

atomic<bool> running(true);
State state = State::ManualControl;

static float prev_odom_x = 0.0f, prev_odom_y = 0.0f;
vector<PathPoint> path;
atomic<bool> telemetry_updated(false);

// Shared telemetry between receiver threads and main loop
struct TelemetryLocal {
    float gy = 0.0f;
    float ds = 0.0f;
    float v  = 0.0f;
    float odom_x = 0.0f;
    float odom_y = 0.0f;
    float odom_th = 0.0f;
    float distances[360];
};
TelemetryLocal sharedTelemetry;
mutex telemMutex;
atomic<bool> telemNew(false);

// Camera shared (BGRA)
mutex camMutex;
cv::Mat camMatBGRA;
atomic<bool> camAvailable(false);

// Chunk reassembly storage
struct ChunkParts {
    uint32_t total_len = 0;
    uint16_t expected = 0;
    vector<pair<int, vector<uint8_t>>> parts;
    chrono::steady_clock::time_point last{chrono::steady_clock::now()};
};
mutex chunksMutex;
unordered_map<uint32_t, ChunkParts> chunkMap;

// Wire headers
const string DEPTH_MAGIC = "WBTD";
const string RGB_MAGIC   = "WBTR";
const string CHNK_MAGIC  = "CHNK";
const string LEGACY_MAGIC= "WBTG";

// Forward declarations
thread path_thread;
void start_path(queue<Msg>* messages);
void getScanPoints(ScanPoint *points, Telemetry &telemetry, Robot &robot);

// Little-endian readers
static inline float read_f32(const uint8_t* p){ float v; memcpy(&v,p,4); return v; }
static inline uint32_t read_u32(const uint8_t* p){ uint32_t v; memcpy(&v,p,4); return v; }
static inline uint16_t read_u16(const uint8_t* p){ uint16_t v; memcpy(&v,p,2); return v; }

// User helpers
void start_path(queue<Msg>* messages){
    path.clear();
    path.push_back({12,-1.25,0});
    path.push_back({11.5,1,1});
    path.push_back({5.5,-3,1});
    path.push_back({3.5,0,1});
    path.push_back({-4,0,1});
    path.push_back({-5,-2,1});
    path.push_back({-7.7,-4,1});
    path.push_back({-7.7,0,0});
    followPath(path, robot, messages);
}

void getScanPoints(ScanPoint *points, Telemetry &telemetry, Robot &robot){
    // Use configurable rotation correction (LIDAR_ROT_RAD)
    for(int i = 0; i < 360; ++i){
        float a = robot.a + (45.0f - i/4.0f) / 57.2958f + LIDAR_ROT_RAD;
        #ifdef BACKWARDS
        a += PI;
        #endif
        float d = telemetry.distances[i];
        float x = d * sin(a) + robot.x;
        float y = -d * cos(a) + robot.y;
        points[i] = {a, d, x, y};
    }
}

// ---------------- robust payload parsing ----------------
static inline size_t find_magic(const vector<uint8_t>& buf, const string &magic, size_t start=0){
    if(buf.size() < magic.size()) return string::npos;
    for(size_t i = start; i + magic.size() <= buf.size(); ++i){
        if(memcmp(buf.data()+i, magic.data(), magic.size()) == 0) return i;
    }
    return string::npos;
}

// handle RGB at offset
size_t handle_rgb_at(const vector<uint8_t>& buf, size_t off){
    if(off + 4 + 2 + 2 + 4 > buf.size()) return 0;
    size_t p = off + 4;
    uint16_t dw = read_u16(buf.data()+p); p+=2;
    uint16_t dh = read_u16(buf.data()+p); p+=2;
    uint32_t ds = read_u32(buf.data()+p); p+=4;
    size_t expect4 = size_t(dw) * size_t(dh) * 4;
    size_t expect3 = size_t(dw) * size_t(dh) * 3;
    if(off + 4 + 2 + 2 + 4 + expect4 <= buf.size()){
        const uint8_t* imgptr = buf.data() + off + 4 + 2 + 2 + 4;
        cv::Mat frame(dh, dw, CV_8UC4, (void*)imgptr);
        {
            lock_guard<mutex> lk(camMutex);
            camMatBGRA = frame.clone();
            camAvailable.store(true);
        }
        cout << "[recv] RGB BGRA " << dw << "x" << dh << " ds=" << ds << endl;
        return 4 + 2 + 2 + 4 + expect4;
    } else if(off + 4 + 2 + 2 + 4 + expect3 <= buf.size()){
        const uint8_t* imgptr = buf.data() + off + 4 + 2 + 2 + 4;
        cv::Mat bgr(dh, dw, CV_8UC3, (void*)imgptr);
        cv::Mat bgra;
        cv::cvtColor(bgr, bgra, cv::COLOR_BGR2BGRA);
        {
            lock_guard<mutex> lk(camMutex);
            camMatBGRA = bgra.clone();
            camAvailable.store(true);
        }
        cout << "[recv] RGB BGR->BGRA " << dw << "x" << dh << " ds=" << ds << endl;
        return 4 + 2 + 2 + 4 + expect3;
    }
    return 0;
}

// Parse header and samples, convert to BGRA image and store.
size_t handle_depth_at(const vector<uint8_t>& buf, size_t off){
    // header: magic(4) | uint16 dw | uint16 dh | int32 ds | float minR | float maxR
    // samples: uint16 per pixel (mm)
    if(off + 4 + 2 + 2 + 4 + 4 + 4 > buf.size()) return 0; // ensure header present
    size_t p = off + 4;
    uint16_t dw = read_u16(buf.data()+p); p += 2;
    uint16_t dh = read_u16(buf.data()+p); p += 2;
    uint32_t ds = read_u32(buf.data()+p); p += 4;
    float minR = read_f32(buf.data()+p); p += 4;
    float maxR = read_f32(buf.data()+p); p += 4;
    size_t expect = size_t(dw) * size_t(dh) * 2;
    if(off + 4 + 2 + 2 + 4 + 4 + 4 + expect > buf.size()) return 0;

    const uint8_t* samples = buf.data() + off + 4 + 2 + 2 + 4 + 4 + 4;

    // Create single-channel 8-bit image mapping depth -> [0..255]
    cv::Mat depth8(dh, dw, CV_8UC1);
    float range = maxR - minR;
    if(range <= 1e-6f) range = 1.0f; // avoid div by zero

    for(uint32_t y = 0; y < dh; ++y){
        uint8_t* row = depth8.ptr<uint8_t>(y);
        const uint8_t* srow = samples + y * dw * 2;
        for(uint32_t x = 0; x < dw; ++x){
            uint16_t mm;
            memcpy(&mm, srow + x*2, 2);
            if(mm == 0){
                row[x] = 0; // invalid -> black
            } else {
                float z = float(mm) / 1000.0f; // meters
                float norm = (z - minR) / range;
                if(norm < 0.0f) norm = 0.0f;
                if(norm > 1.0f) norm = 1.0f;
                uint8_t v = uint8_t((1.0f - norm) * 255.0f); // nearer = brighter
                row[x] = v;
            }
        }
    }

    // Convert to BGRA for rendering
    cv::Mat bgra;
    cv::cvtColor(depth8, bgra, cv::COLOR_GRAY2BGRA);

    {
        lock_guard<mutex> lk(camMutex);
        camMatBGRA = bgra.clone();
        camAvailable.store(true);
    }

    cout << "[recv] DEPTH " << dw << "x" << dh << " ds=" << ds << " minR=" << minR << " maxR=" << maxR << " bytes=" << expect << endl;
    return 4 + 2 + 2 + 4 + 4 + 4 + expect;
}

// legacy telemetry handler
size_t handle_legacy_at(const vector<uint8_t>& buf, size_t off){
    if(off + 4 + 9*4 + 4 > buf.size()) return 0;
    size_t p = off + 4;
    float f9[9];
    for(int i=0;i<9;i++){ f9[i] = read_f32(buf.data()+p + i*4); }
    p += 9*4;
    uint32_t n = read_u32(buf.data()+p); p += 4;
    if(off + 4 + 9*4 + 4 + n*4 > buf.size()) return 0;
    TelemetryLocal t;
    t.odom_x = f9[0]; t.odom_y = f9[1]; t.odom_th = f9[2]; t.v = f9[3];
    t.gy = f9[8];
    size_t copy_n = min<uint32_t>(n, 360u);
    for(size_t i=0;i<copy_n;i++) t.distances[i] = read_f32(buf.data()+p + i*4);
    {
        lock_guard<mutex> lk(telemMutex);
        t.ds = distance(t.odom_x, t.odom_y, prev_odom_x, prev_odom_y) * ENCODER_LINEAR_MULTIPLIER;
        prev_odom_x = t.odom_x; prev_odom_y = t.odom_y;
        sharedTelemetry = t;
        telemNew.store(true);
        telemetry_updated.store(true);
    }
    cout << "[recv] LEGACY telemetry n=" << n << " odom=(" << t.odom_x << "," << t.odom_y << ")" << endl;
    return 4 + 9*4 + 4 + n*4;
}

// top-level payload processor: searches for known magic messages inside 'payload' and processes them.
void process_payload(vector<uint8_t>& payload){
    size_t offset = 0;
    size_t B = payload.size();
    while(offset + 4 <= B){
        size_t i_rgb = find_magic(payload, RGB_MAGIC, offset);
        size_t i_depth = find_magic(payload, DEPTH_MAGIC, offset);
        size_t i_legacy = find_magic(payload, LEGACY_MAGIC, offset);

        size_t next = string::npos;
        if(i_rgb != string::npos) next = (next==string::npos? i_rgb : min(next,i_rgb));
        if(i_depth != string::npos) next = (next==string::npos? i_depth : min(next,i_depth));
        if(i_legacy != string::npos) next = (next==string::npos? i_legacy : min(next,i_legacy));

        if(next == string::npos) break;
        if(next > offset){ offset = next; continue; }

        size_t consumed = 0;
        if(memcmp(payload.data()+offset, RGB_MAGIC.data(), 4) == 0){
            consumed = handle_rgb_at(payload, offset);
            if(consumed == 0) break;
        } else if(memcmp(payload.data()+offset, DEPTH_MAGIC.data(), 4) == 0){
            consumed = handle_depth_at(payload, offset);
            if(consumed == 0) break;
        } else if(memcmp(payload.data()+offset, LEGACY_MAGIC.data(), 4) == 0){
            consumed = handle_legacy_at(payload, offset);
            if(consumed == 0) break;
        } else {
            offset += 1;
            continue;
        }
        offset += consumed;
    }
}

// Chunk reassembly
void process_chunk_and_maybe_reassemble(const vector<uint8_t>& incoming){
    if(incoming.size() >= 4 && memcmp(incoming.data(), CHNK_MAGIC.data(), 4) == 0){
        size_t off = 4;
        if(incoming.size() < off + 4 + 4 + 2 + 2) return;
        uint32_t msg_id = read_u32(incoming.data()+off); off += 4;
        uint32_t total_len = read_u32(incoming.data()+off); off += 4;
        uint16_t idx = read_u16(incoming.data()+off); off += 2;
        uint16_t count = read_u16(incoming.data()+off); off += 2;
        vector<uint8_t> part(incoming.begin()+off, incoming.end());

        lock_guard<mutex> lk(chunksMutex);
        auto &entry = chunkMap[msg_id];
        entry.last = chrono::steady_clock::now();
        entry.total_len = total_len;
        entry.expected = count;
        entry.parts.emplace_back((int)idx, std::move(part));
        if(entry.parts.size() >= entry.expected){
            sort(entry.parts.begin(), entry.parts.end(), [](auto &a, auto &b){ return a.first < b.first; });
            vector<uint8_t> full; full.reserve(entry.total_len);
            for(auto &p : entry.parts) full.insert(full.end(), p.second.begin(), p.second.end());
            chunkMap.erase(msg_id);
            cout << "[recv] assembled chunk msg_id=" << msg_id << " len=" << full.size() << endl;
            process_payload(full);
        }
        for(auto it = chunkMap.begin(); it != chunkMap.end(); ){
            if(chrono::steady_clock::now() - it->second.last > 5s) it = chunkMap.erase(it);
            else ++it;
        }
    } else {
        vector<uint8_t> v = incoming;
        process_payload(v);
    }
}


void udp_listener(const string &port){
    try {
        asio::io_context io;
        udp::socket sock(io, udp::endpoint(udp::v4(), stoi(port)));
        vector<uint8_t> buf(65536);
        udp::endpoint sender;
        while(running.load()){
            error_code ec;
            size_t len = sock.receive_from(asio::buffer(buf), sender, 0, ec);
            if(ec) { this_thread::sleep_for(1ms); continue; }
            vector<uint8_t> payload(buf.begin(), buf.begin()+len);
            process_chunk_and_maybe_reassemble(payload);
        }
    } catch(const std::exception &e){
        cerr << "[udp_listener] exception: " << e.what() << endl;
    }
}


void tcp_listener(const string &host, const string &port){
    try {
        asio::io_context io;
        tcp::acceptor acceptor(io, tcp::endpoint(asio::ip::make_address(host), stoi(port)));
        while(running.load()){
            tcp::socket sock(io);
            error_code ec;
            acceptor.accept(sock, ec);
            if(ec){ this_thread::sleep_for(100ms); continue; }
            while(running.load()){
                uint32_t len_le = 0;
                error_code read_ec;
                size_t n = asio::read(sock, asio::buffer(&len_le, sizeof(len_le)), read_ec);
                if(read_ec || n != sizeof(len_le)) break;
                uint32_t len = len_le;
                if(len == 0 || len > 10*1024*1024) break;
                vector<uint8_t> payload(len);
                size_t got = 0;
                while(got < len){
                    error_code part_ec;
                    size_t r = sock.read_some(asio::buffer(payload.data()+got, len-got), part_ec);
                    if(part_ec) { read_ec = part_ec; break; }
                    got += r;
                }
                if(got != len) break;
                process_chunk_and_maybe_reassemble(payload);
            }
            error_code ignored;
            sock.shutdown(asio::socket_base::shutdown_both, ignored);
            sock.close(ignored);
        }
    } catch(const std::exception &e){
        cerr << "[tcp_listener] exception: " << e.what() << endl;
    }
}


void draw_loop(){
    bool path_thread_exists = false;
    const int winW = GRID_W, winH = GRID_H;
    InitWindow(winW, winH, "ArchBTW monitoring + Camera");
    SetTargetFPS(60);

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

    Texture2D camTex{0,0,0,0,0};
    int camW = 0, camH = 0;

    cv::Mat distanceGrid(GRID_H, GRID_W, CV_32F);
    PathPoint tmpGoal{-7.7f, 0.0f, 0};
    cv::Point goal = worldToGrid(tmpGoal);

    for (int y = 0; y < GRID_H; ++y) {
        float dy = float(y - goal.y);
        for (int x = 0; x < GRID_W; ++x) {
            float dx = float(x - goal.x);
            distanceGrid.at<float>(y, x) = sqrt(dx*dx + dy*dy);
        }
    }
    double minVal, maxVal;
    cv::minMaxLoc(distanceGrid, &minVal, &maxVal);
    distanceGrid = (distanceGrid - minVal) / (maxVal - minVal);

    cv::Mat colorGrid(GRID_H, GRID_W, CV_8UC3);
    for (int y = 0; y < GRID_H; ++y) {
        const float* distRow = distanceGrid.ptr<float>(y);
        for (int x = 0; x < GRID_W; ++x) {
            float val = distRow[x];
            cv::Vec3b &px = colorGrid.at<cv::Vec3b>(y,x);
            px[0] = (uchar)(val * 255);
            px[1] = 0;
            px[2] = (uchar)((1.0f - val) * 255);
        }
    }
    cv::Mat colorGridRGBA;
    cv::cvtColor(colorGrid, colorGridRGBA, cv::COLOR_BGR2BGRA);
    UpdateTexture(colorTex, colorGridRGBA.data);

    double last_log = 0.0;

    while(!WindowShouldClose()){
        if(state == State::ManualControl) handle_wasd();

        if(IsKeyPressed(KEY_P)){
            if(!path_thread_exists){
                path_thread = thread(start_path, &message_queue);
                path_thread_exists = true;
            } else {
                message_queue.push(Msg::STOPFOLOW);
            }
        }

        // camera update (main thread)
        if(camAvailable.load()){
            cv::Mat local;
            { lock_guard<mutex> lk(camMutex); if(!camMatBGRA.empty()) local = camMatBGRA.clone(); camAvailable.store(false); }
            if(!local.empty()){
                // local is BGRA (4 channels)
                // cv::flip(local, local, 0);
                cv::Mat rgba;
                cv::cvtColor(local, rgba, cv::COLOR_BGRA2RGBA);
                if(camW != rgba.cols || camH != rgba.rows){
                    if(camTex.id) UnloadTexture(camTex);
                    Image tmp = GenImageColor(rgba.cols, rgba.rows, BLANK);
                    tmp.format = PIXELFORMAT_UNCOMPRESSED_R8G8B8A8;
                    camTex = LoadTextureFromImage(tmp);
                    UnloadImage(tmp);
                    camW = rgba.cols; camH = rgba.rows;
                }
                if(camTex.id) UpdateTexture(camTex, rgba.data);
            }
        }

        BeginDrawing();
        ClearBackground(RAYWHITE);

        cv::mixChannels(grid, pixelsMat, {0,0});
        cv::mixChannels(pathfind_grid, pixelsMat, {0,1});
        UpdateTexture(gridTex, pixels);

        BeginShaderMode(shader);
        SetShaderValueTexture(shader, GetShaderLocation(shader, "uColorMap"), colorTex);
        DrawTexture(gridTex, 0, 0, WHITE);
        EndShaderMode();

        for(size_t i = 1; i < path.size(); ++i){
            cv::Point2f p1 = worldToGrid(path[i-1]);
            cv::Point2f p2 = worldToGrid(path[i]);
            DrawLineEx({p1.x, p1.y}, {p2.x, p2.y}, 3, BLUE);
        }

        DrawCircle(goal.x, goal.y, 5, MAGENTA);

        float screen_robot_x = robot.x / CELL_SIZE + GRID_W/2;
        float screen_robot_y = robot.y / CELL_SIZE + GRID_H/2;
        float dir_x = 10 * sin(robot.a);
        float dir_y = -10 * cos(robot.a);
        DrawCircle(screen_robot_x, screen_robot_y, 10, GREEN);
        DrawLineEx({screen_robot_x, screen_robot_y}, {screen_robot_x+dir_x, screen_robot_y+dir_y}, 3, BLACK);

        DrawText(TextFormat("X: %.2f", robot.x), 10, 10, 20, RED);
        DrawText(TextFormat("Y: %.2f", robot.y), 10, 30, 20, RED);
        DrawText(TextFormat("%.02f FPS", 1.0/GetFrameTime()), 10, 50, 20, RED);

        if(camTex.id){
            int camWdraw = camTex.width / 2;
            int camHdraw = camTex.height / 2;
            int x = winW - camWdraw - 10;
            int y = 10;
            Rectangle src = {0.0f, 0.0f, (float)camTex.width, (float)camTex.height};
            Rectangle dst = {(float)x, (float)y, (float)camWdraw, (float)camHdraw};
            DrawTexturePro(camTex, src, dst, {0,0}, 0.0f, WHITE);
            DrawRectangleLines(x-1, y-1, camWdraw+2, camHdraw+2, BLACK);
        } else {
            DrawText("No camera", winW - 110, 10, 20, DARKGRAY);
        }

        EndDrawing();

        if(GetTime() - last_log > 1.0){
            last_log = GetTime();
            TelemetryLocal copy;
            { lock_guard<mutex> lk(telemMutex); copy = sharedTelemetry; }
            cout << "[ui] ODOM(x=" << copy.odom_x << ", y=" << copy.odom_y << ", θ=" << copy.odom_th << ") "
                 << "v=" << copy.v << " ds=" << copy.ds << " gy=" << copy.gy << " lidar_rot_deg=" << LIDAR_ROT_DEG << endl;
        }
    }

    running.store(false);
    delete[] pixels;
    if(camTex.id) UnloadTexture(camTex);
    UnloadTexture(gridTex);
    UnloadTexture(colorTex);
    UnloadShader(shader);
    CloseWindow();
}


int main(int argc, char** argv){
    string host = "0.0.0.0", port = "5600";
    if(const char* h = getenv("TEL_HOST")) host = h;
    if(const char* p = getenv("TEL_PORT")) port = p;
    string proto = "tcp";
    if(const char* q = getenv("TEL_PROTO")) proto = q;

    cout << "telemetry host: " << host << ":" << port << " proto=" << proto
         << " LIDAR_ROT_DEG=" << LIDAR_ROT_DEG << endl;

    // Camera is Udp
    thread udp_thread(udp_listener, port);

    // Tcp for legacy
    thread tcp_thread;
    if(proto == "tcp"){
        tcp_thread = thread(tcp_listener, host, port);
    }

    init_movement();

    #ifdef VISUALIZATION
    thread ui_thread(draw_loop);
    #else
    path_thread = thread(start_path, &message_queue);
    #endif

    Telemetry telemetry;

    while(running.load()){
        if(telemNew.load()){
            lock_guard<mutex> lk(telemMutex);
            telemetry.gy = sharedTelemetry.gy;
            telemetry.ds = sharedTelemetry.ds;
            telemetry.v  = sharedTelemetry.v;
            for(int i=0;i<360;i++) telemetry.distances[i] = sharedTelemetry.distances[i];
            telemNew.store(false);
            telemetry_updated.store(true);
        }

        if(!telemetry_updated.load()){
            this_thread::sleep_for(2ms);
            continue;
        }

        // dead reckoning
        robot.a -= telemetry.gy * DT;
        robot.x += telemetry.ds * sin(robot.a);
        robot.y -= telemetry.ds * cos(robot.a);
        robot.v = telemetry.v;
        telemetry_updated.store(false);

        // mapping update
        cv::Mat1b gridCopy = grid.clone();
        ScanPoint scanPoints[360];
        getScanPoints(scanPoints, telemetry, robot);

        for(int i = 3; i < 358; ++i){
            cv::Point trianglePoints[3] = {
                worldToGrid(robot.x, robot.y),
                worldToGrid(scanPoints[i-1]),
                worldToGrid(scanPoints[i])
            };
            cv::fillConvexPoly(gridCopy, trianglePoints, 3, 2);
        }

        for(int i = 1; i < 360; ++i){
            if(scanPoints[i].d < 8 && scanPoints[i-1].d < 8 &&
               distance(scanPoints[i-1], scanPoints[i]) < 0.25f){
                cv::line(gridCopy, worldToGrid(scanPoints[i-1]), worldToGrid(scanPoints[i]), 1);
            }
        }

        gridCopy.copyTo(grid);
        cv::compare(gridCopy, 1, gridCopy, cv::CMP_EQ);
        int dilation_size = 10;
        cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE,
                        cv::Size(2*dilation_size + 1, 2*dilation_size + 1),
                        cv::Point(dilation_size, dilation_size));
        cv::dilate(gridCopy, pathfind_grid, element);
    }

    running.store(false);
    if(udp_thread.joinable()) udp_thread.join();
    if(tcp_thread.joinable()) tcp_thread.join();
    #ifdef VISUALIZATION
    if(path_thread.joinable()) path_thread.join();
    #endif
    return 0;
}
