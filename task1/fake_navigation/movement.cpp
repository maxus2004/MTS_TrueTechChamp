#include "movement.h"

void handle_input(asio::ip::udp::socket& s){
    if(IsKeyPressed(KEY_A)) handle_move(MoveDirection::LEFT, s);
    if(IsKeyPressed(KEY_D)) handle_move(MoveDirection::RIGHT, s);
    if(IsKeyPressed(KEY_W)) handle_move(MoveDirection::UP, s);
    if(IsKeyPressed(KEY_S)) handle_move(MoveDirection::DOWN, s);
}


void handle_move(MoveDirection d, asio::ip::udp::socket& s){
    float v,w;
    switch (d) {
        case MoveDirection::UP: v = 0.5; w = 0; break;
        case MoveDirection::DOWN: v = -0.5; w = 0; break;
        case MoveDirection::LEFT: v = 0; w = 0.5; break;
        case MoveDirection::RIGHT: v = 0; w = -0.5; break;
    }   

    std::array<char, sizeof(float) * 2> packet;

    std::memcpy(packet.data(), &v, sizeof(float));
    std::memcpy(packet.data() + sizeof(float), &w, sizeof(float));

    s.send_to(asio::buffer(packet), asio::ip::udp::endpoint(asio::ip::make_address("127.0.0.1"), 5555));
}