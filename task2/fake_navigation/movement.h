#pragma once

#include <asio.hpp>
#include <raylib.h>
#include "utils.h"

#include <asio/ip/udp.hpp>

using asio::ip::tcp;
using namespace std;

void init_movement();
void handle_wasd();
void send_move(float v, float w);