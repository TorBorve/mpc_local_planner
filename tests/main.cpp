#include "testBounds.h"
#include "testMpc.h"

#include <iostream>

int main(int argc, char **argv)
{
    using namespace mpc;
    ros::init(argc, argv, "tesMpc");
    std::cout << "Testing bounds:\n";
    testBounds();
    std::cout << "Finished testing!\n";
    testMpc();

    return 0;
}