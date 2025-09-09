#include <asio.hpp>
#include <iostream>
#include <cmath>
#include <raylib.h>

using asio::ip::tcp;
using namespace std;

//cell grid parameters
#define GRID_W 500
#define GRID_H 500
#define CELL_SIZE 0.02f // 1 cell = 20mm

//odometry unfucking parameters
#define ENCODER_LINEAR_MULTIPLIER 1
#define ENCODER_ANGULAR_MULTIPLIER 1


struct mts_telemetry_packet_t {
    char header[8];
    float x;
    float y;
    float a;
    float vx;
    float vy;
    float vth;
    uint32_t lidar_count;
    float distances[360];
};

struct telemetry_t {
    float ds;
    float da;
    float distances[360];
};

uint8_t grid[GRID_W][GRID_H]; 

float robot_x = 0;
float robot_y = 0;
float robot_a = 0;

bool running = true;

float prev_mts_x;
float prev_mts_y;
float prev_mts_a;

void update_telemetry(telemetry_t* telemetry, tcp::socket* telemetry_socket){
    //get telemetry data
    std::array<char, 8192> buf;
    size_t len = telemetry_socket->read_some(asio::buffer(buf));
    mts_telemetry_packet_t mts_telemetry_packet;
    memcpy(&mts_telemetry_packet, buf.data(), sizeof(mts_telemetry_packet));

    //get actual odometry data from fucked up telemetry
    telemetry->da = -(mts_telemetry_packet.a-prev_mts_a);
    telemetry->ds = sqrt(pow(mts_telemetry_packet.x-prev_mts_x,2)+pow(mts_telemetry_packet.y-prev_mts_y,2));
    //copy lidar data to telemetry variable
    memcpy(&(telemetry->distances),&(mts_telemetry_packet.distances),sizeof(telemetry->distances));

    //update prev values
    prev_mts_x = mts_telemetry_packet.x;
    prev_mts_y = mts_telemetry_packet.y;
    prev_mts_a = mts_telemetry_packet.a;
}

void draw_loop() {
    InitWindow(GRID_W, GRID_H, "ArchBTW monitoring");

    while (!WindowShouldClose()) {
        BeginDrawing();

        //draw map cells
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

        //draw robot
        float screen_robot_x = robot_x/CELL_SIZE+GRID_W/2;
        float screen_robot_y = robot_y/CELL_SIZE+GRID_H/2;
        float dir_x = 10*sin(robot_a);
        float dir_y = -10*cos(robot_a);
        DrawCircle(screen_robot_x, screen_robot_y, 10, GREEN);
        DrawLineEx({screen_robot_x, screen_robot_y}, {screen_robot_x+dir_x, screen_robot_y+dir_y}, 3, BLACK);

        EndDrawing();
    }

    running = false;
}

int main() {
    //connect to simulation
    asio::io_context io_context;
    tcp::acceptor acceptor(io_context, tcp::endpoint(tcp::v4(), 5600));
    tcp::socket telemetry_socket(io_context);
    acceptor.accept(telemetry_socket);

    thread draw_thread(draw_loop);

    telemetry_t telemetry;

    while (running) {
        update_telemetry(&telemetry, &telemetry_socket);

        //dead reckoning (The missle knows where it is because....)
        robot_a += telemetry.da;
        robot_x += telemetry.ds*sin(robot_a);
        robot_y -= telemetry.ds*cos(robot_a);

        //obstacle mapping
        for(int i = 0;i<360;i++){
            float d = telemetry.distances[i];
            float a = robot_a+(45-i/4)/57.2958f;
            float rel_x = d*sin(a);
            float rel_y = -d*cos(a);
            //fill while cells where there are no obstacles
            for(int j = 0;j<d/CELL_SIZE;j++){
                float x = robot_x+rel_x/(d/CELL_SIZE)*j;
                float y = robot_y+rel_y/(d/CELL_SIZE)*j;
                int cell_x = x/CELL_SIZE+GRID_W/2;
                int cell_y = y/CELL_SIZE+GRID_H/2;
                if(cell_x<0 || cell_x>GRID_W || cell_y<0 || cell_y>GRID_H)continue;
                grid[cell_x][cell_y] = 2;
            }
            //set black cell if found an obstacle
            if(d<8){
                float x = robot_x+rel_x;
                float y = robot_y+rel_y;
                int cell_x = x/CELL_SIZE+GRID_W/2;
                int cell_y = y/CELL_SIZE+GRID_H/2;
                if(cell_x<0 || cell_x>GRID_W || cell_y<0 || cell_y>GRID_H)continue;
                grid[cell_x][cell_y] = 1;
            }
        }
    }

    draw_thread.join();
}
