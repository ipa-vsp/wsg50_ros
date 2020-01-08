#include "wsg_50_tcp_driver/function.h"

int main(int argc, char** argv) {
    std::string addr = "12.15.51.1";
    unsigned short port = 22;
    iwtros::function test(addr.c_str(), port);
    return 0;
}