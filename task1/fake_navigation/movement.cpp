// movement.cpp  - UART/serial replacement for UDP-based movement control
// Sends JSON commands over serial like: {"T":1,"L":<left>,"R":<right>}\r\n

#include "movement.h"

#include <asio.hpp>
#include <iostream>
#include <thread>
#include <mutex>
#include <atomic>
#include <memory>
#include <sstream>
#include <cmath>
#include <cstdlib>

using namespace std;

extern bool telemetry_updated; // defined in main.cpp

asio::io_context io_context;                       // io_context used for serial reads/writes in this module
static std::unique_ptr<asio::serial_port> serial_port;   // opened by init_movement()
static std::thread io_thread;                             // runs io_context
static std::mutex write_mutex;                            // protect writes to serial
static std::atomic<bool> serial_ok{false};


static const int DEFAULT_BAUD = 115200;
static const char *DEFAULT_DEV = "/dev/ttyUSB0";
static const float SCALE = 100.0f;   // scale (v,w) into wheel command range
static const int MIN_CMD = -100;
static const int MAX_CMD = 100;

static inline int clamp_int(int v, int lo, int hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static void start_async_read(); // forward

void init_movement(){
    const char *dev_env = getenv("CMD_UART");
    const char *baud_env = getenv("CMD_BAUD");

    string dev = dev_env ? dev_env : DEFAULT_DEV;
    int baud = DEFAULT_BAUD;
    if(baud_env){
        try{
            baud = stoi(baud_env);
        } catch(...){ baud = DEFAULT_BAUD; }
    }

    cout << "movement: opening serial " << dev << " @ " << baud << endl;

    try {
        serial_port = make_unique<asio::serial_port>(io_context);
        serial_port->open(dev);
        serial_port->set_option(asio::serial_port_base::baud_rate(baud));
        // Optional serial settings can be adjusted here
        // serial_port->set_option(asio::serial_port_base::parity(asio::serial_port_base::parity::none));
        // serial_port->set_option(asio::serial_port_base::character_size(8));
        // serial_port->set_option(asio::serial_port_base::flow_control(asio::serial_port_base::flow_control::none));
        serial_ok = true;
    } catch (std::exception &e) {
        cerr << "movement: failed to open serial: " << e.what() << endl;
        serial_ok = false;
        return;
    }

    // Start asynchronous read handler (prints incoming lines)
    start_async_read();

    // Run the io_context in a background thread so async reads keep working
    io_thread = std::thread([](){
        try {
            io_context.run();
        } catch (std::exception &e) {
            cerr << "movement: io_context error: " << e.what() << endl;
        }
    });
}

static void start_async_read(){
    if (!serial_port || !serial_port->is_open()) return;
    auto buf_ptr = std::make_shared<asio::streambuf>();

    asio::async_read_until(*serial_port, *buf_ptr, '\n',
        [buf_ptr](const asio::error_code& ec, std::size_t bytes_transferred) {
            if (ec) {
                if (ec == asio::error::operation_aborted) return;
                cerr << "movement: serial read error: " << ec.message() << endl;
                serial_ok = false;
                return;
            }

            // Extract line from buffer
            std::istream is(buf_ptr.get());
            std::string line;
            std::getline(is, line);
            if (!line.empty()) {
                cout << "[UART RX] " << line << endl;
            }

            // Re-arm read for next line
            start_async_read();
        }
    );
}

void send_move(float v, float w){
    #ifdef BACKWARDS
    v = -v;
    #endif

    // mapping used: L = (v - w) * SCALE, R = (v + w) * SCALE
    float left_f = (v - w) * SCALE;
    float right_f = (v + w) * SCALE;

    int L = clamp_int((int)std::lround(left_f), MIN_CMD, MAX_CMD);
    int R = clamp_int((int)std::lround(right_f), MIN_CMD, MAX_CMD);

    // Format JSON exactly as Python example and terminate with CRLF
    std::ostringstream oss;
    oss << "{\"T\":1,\"L\":" << L << ",\"R\":" << R << "}\r\n";
    std::string out = oss.str();

    if(!serial_ok){
        static bool warned = false;
        if(!warned){
            cerr << "movement: serial not available, cannot send: " << out;
            warned = true;
        }
        return;
    }

    std::lock_guard<std::mutex> lk(write_mutex);
    try {
        asio::write(*serial_port, asio::buffer(out.data(), out.size()));
        // Logging
        cout << "[UART TX] " << out;
    } catch (std::exception &e) {
        cerr << "movement: serial write error: " << e.what() << endl;
        serial_ok = false;
    }
}

void handle_wasd(){
    if(!telemetry_updated) return;
    telemetry_updated = false;

    float v = 0.0f;
    float w = 0.0f;
    if(IsKeyDown(KEY_A)) w += 0.5f;
    if(IsKeyDown(KEY_D)) w -= 0.5f;
    if(IsKeyDown(KEY_W)) v += 0.5f;
    if(IsKeyDown(KEY_S)) v -= 0.5f;
    send_move(v, w);
}

void shutdown_movement(){
    try {
        if (serial_port && serial_port->is_open()) {
            // send stop command once
            {
                std::lock_guard<std::mutex> lk(write_mutex);
                string stop = "{\"T\":1,\"L\":0,\"R\":0}\r\n";
                asio::write(*serial_port, asio::buffer(stop.data(), stop.size()));
            }
            serial_port->close();
        }
        io_context.stop();
        if (io_thread.joinable()) io_thread.join();
    } catch(...) {
        // swallow exceptions during shutdown
    }
}
