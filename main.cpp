#include "Engine.hpp"
#include <iostream>
#include <thread>
int main()
{
    std::shared_ptr<Engine> e(new Engine);

    std::thread serverThread(&Engine::startServer, e, "0.0.0.0", 8080);

    e->observeQueue();
    serverThread.join();
    return 0;
}
