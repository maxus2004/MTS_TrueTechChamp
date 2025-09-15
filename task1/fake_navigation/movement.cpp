#include "movement.h"
#include <iostream>
using namespace std;

void handle_input(asio::ip::udp::socket& s){

    float v = 0;
    float w = 0;
    if(IsKeyDown(KEY_A)) w += 0.5;
    if(IsKeyDown(KEY_D)) w -= 0.5;
    if(IsKeyDown(KEY_W)) v += 0.5;
    if(IsKeyDown(KEY_S)) v -= 0.5;
    cout << v << endl;
    send_move(v, w, s);
}


void send_move(float v, float w , asio::ip::udp::socket& s){
    std::array<char, sizeof(float) * 2> packet;

    std::memcpy(packet.data(), &v, sizeof(float));
    std::memcpy(packet.data() + sizeof(float), &w, sizeof(float));

    s.send_to(asio::buffer(packet), asio::ip::udp::endpoint(asio::ip::make_address("127.0.0.1"), 5555));
}