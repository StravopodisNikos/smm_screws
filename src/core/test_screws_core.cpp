#include <iostream>
#include "smm_screws/core/ScrewsDynamics.h"

// JUST CHECK CORE:
// nikos@nikos-pc2:~/ros2_ws/src/smm_screws/src/core 
// $ g++ -std=c++17 -Wall -Wextra     -I../../include     -I../../include/smm_screws/core     -I/usr/include/eigen3     -c ScrewsMain.cpp ScrewsKinematics.cpp ScrewsDynamics.cpp

// FOR COMPILE & TEST:
// nikos@nikos-pc2:~/ros2_ws/src/smm_screws/src/core
// $g++ -std=c++17 -Wall -Wextra \
//    -I../../include \
//    -I../../include/smm_screws/core \
//    -I/usr/include/eigen3 \
//    ScrewsMain.cpp \
//    ScrewsKinematics.cpp \
//    ScrewsDynamics.cpp \
//    test_screws_core.cpp \
//    -o test_screws_core

int main() {
    std::cout << "Screws core compiles and links.\n";
    return 0;
}

