#include "movement.h"
#include <iostream>
using namespace std;

extern bool telemetry_updated;

asio::io_context io_context;
asio::ip::udp::socket control_socket(io_context, asio::ip::udp::v4());

void init_movement(){
    string host = "127.0.0.1";
    string port = "5555";
    if(getenv("CMD_HOST") != NULL){
        host = getenv("CMD_HOST");
    }
    if(getenv("CMD_PORT") != NULL){
        port = getenv("CMD_PORT");
    }
    cout << "control host: " << host << ":" << port << endl;
    control_socket.connect(asio::ip::udp::resolver(io_context).resolve(host, port).begin()->endpoint());
}

void handle_wasd(){
    if(!telemetry_updated) return;
    telemetry_updated = false;

    float v = 0;
    float w = 0;
    if(IsKeyDown(KEY_A)) w += 0.5;
    if(IsKeyDown(KEY_D)) w -= 0.5;
    if(IsKeyDown(KEY_W)) v += 0.5;
    if(IsKeyDown(KEY_S)) v -= 0.5;
    send_move(v, w);
}


void send_move(float v, float w){
    std::array<char, sizeof(float) * 2> packet;

    #ifdef BACKWARDS
    v = -v;
    #endif

    std::memcpy(packet.data(), &v, sizeof(float));
    std::memcpy(packet.data() + sizeof(float), &w, sizeof(float));
    control_socket.send(asio::buffer(packet));
}