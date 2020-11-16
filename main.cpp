#include "Engine.hpp"
#include <iostream>
#include <thread>
int main()
{

    // Engine e;

    // e.runEngine();
    std::shared_ptr<Engine> e(new Engine);
    e->parseJsonString("layer.json");
    std::cout << "Done reading! " << std::endl;

    std::thread serverThread(&Engine::startServer, e, "0.0.0.0", 8080);

    e->observeQueue();
    serverThread.join();
    return 0;
}
