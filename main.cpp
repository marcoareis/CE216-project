#include "class1.h"
#include <iostream>
#include <sstream>

int main() {
    One one; 
    
    while (0) {
        one.x_rand();
        std::cout << "x=" << one.getX() << std::endl;
    }

    std::cout << (-1 % 180) << "," << (-2 % 180) <<  std::endl;

    std::ostringstream ss;
    for(int i = 0; i < 6; i++) {
        ss << std::string("leg") <<  i;
        std::cout << ss.str() << std::endl;
        ss.str("");
    }
    return 0;
}
