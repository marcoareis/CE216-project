#include "class1.h"
#include <iostream>


int main() {
    One one; 
    
    while (0) {
        one.x_rand();
        std::cout << "x=" << one.getX() << std::endl;
    }

    std::cout << (-1 % 180) << "," << (-2 % 180) <<  std::endl;
    return 0;
}