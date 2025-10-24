#pragma once
#include <raylib.h>

void init_movement();

void handle_wasd();

void send_move(float v, float w);

void shutdown_movement();
