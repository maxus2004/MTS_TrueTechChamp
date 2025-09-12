#pragma once

#include <asio.hpp>
#include <raylib.h>
#include "utils.h"

#include <asio/ip/udp.hpp>

using asio::ip::tcp;
using namespace std;

void handle_input(asio::ip::udp::socket& s);
void handle_move(MoveDirection d, asio::ip::udp::socket& s);