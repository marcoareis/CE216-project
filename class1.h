#ifndef ONE
#define ONE
#include <cstdlib>
class One {
    public:
        double getX() { return x;}
        void x_rand() {x = rand();} 
    private:
        double x;
};
#endif
