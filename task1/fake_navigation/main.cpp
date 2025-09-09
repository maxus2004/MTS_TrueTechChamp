#include <asio.hpp>
#include <iostream>
#include <math.h>
#include "raylib.h"

using asio::ip::tcp;
using namespace std;

struct telemetry_packet_t {
    char header[8];
    float odom_x;
    float odom_y;
    float odom_th;
    float vx;
    float vy;
    float vth;
    uint32_t lidar_count;
    float lidar_distances[360];
};

#define GRID_W 500
#define GRID_H 500
#define CELL_SIZE 0.02f // 1 cell = 20mm
uint8_t grid[GRID_W][GRID_H]; 
float robot_x = 0;
float robot_y = 0;
float robot_a = 0;

bool running = true;

void draw_loop() {
    InitWindow(GRID_W, GRID_H, "ArchBTW monitoring");

    while (!WindowShouldClose()) {
        BeginDrawing();
        for (int x = 0; x < GRID_W; x++) {
            for (int y = 0; y < GRID_H; y++) {
                switch (grid[x][y]) {
                    case 0:
                        DrawPixel(x, y, GRAY);
                        break;
                    case 1:
                        DrawPixel(x, y, BLACK);
                        break;
                    case 2:
                        DrawPixel(x, y, WHITE);
                        break;
                }
            }
        }

        DrawCircle(robot_x/CELL_SIZE+GRID_W/2,robot_y/CELL_SIZE+GRID_H/2,10,GREEN);
        float dirx = 10*sinf(robot_a);
        float diry = -10*cosf(robot_a);
        DrawLineEx(
            {robot_x/CELL_SIZE+GRID_W/2,robot_y/CELL_SIZE+GRID_H/2},
            {robot_x/CELL_SIZE+GRID_W/2+dirx, robot_y/CELL_SIZE+GRID_H/2+diry},
            3,BLACK
        );
        EndDrawing();
    }

    running = false;
}

int main() {
    asio::io_context io_context;
    tcp::acceptor acceptor(io_context, tcp::endpoint(tcp::v4(), 5600));
    tcp::socket socket(io_context);
    acceptor.accept(socket);

    std::array<char, 8192> buf;
    asio::error_code error;

    telemetry_packet_t telemetry;

    thread draw_thread(draw_loop);

    while (running) {
        size_t len = socket.read_some(asio::buffer(buf), error);
        memcpy(&telemetry, buf.data(), sizeof(telemetry));
        robot_x = -telemetry.odom_y;
        robot_y = -telemetry.odom_x;
        robot_a = -telemetry.odom_th;

        for(int i = 0;i<360;i++){
            float d = telemetry.lidar_distances[i];
            cout << d << endl;
            if(d>=8)continue;
            float a = robot_a+(45-i/4)/57.2958f;
            float rel_x = d*sinf(a);
            float rel_y = -d*cosf(a);
            for(int j = 0;j<d/CELL_SIZE;j++){
                float x = robot_x+rel_x/(d/CELL_SIZE)*j;
                float y = robot_y+rel_y/(d/CELL_SIZE)*j;
                int cell_x = x/CELL_SIZE+GRID_W/2;
                int cell_y = y/CELL_SIZE+GRID_H/2;
                if(cell_x<0 || cell_x>GRID_W || cell_y<0 || cell_y>GRID_H)continue;
                grid[cell_x][cell_y] = 2;
            }
            float x = robot_x+rel_x;
            float y = robot_y+rel_y;
            int cell_x = x/CELL_SIZE+GRID_W/2;
            int cell_y = y/CELL_SIZE+GRID_H/2;
            if(cell_x<0 || cell_x>GRID_W || cell_y<0 || cell_y>GRID_H)continue;
            grid[cell_x][cell_y] = 1;
        }
    }
}
