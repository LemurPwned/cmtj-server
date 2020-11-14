#include "Engine.hpp"
#include <iostream>

int main()
{

    Engine e;
    e.parseJsonString("layer.json");
    std::cout << "Done reading! " << std::endl;
    return 0;
}
