// Demo of vector plot.
// Compile it with:
//   g++ -o example-vector example-vector.cc -lboost_iostreams -lboost_system -lboost_filesystem

#include <vector>
#include <cmath>
#include <boost/tuple/tuple.hpp>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo.hh>
#include <iostream>

#include "gnuplot-iostream.h"

#include <unistd.h>
inline void mysleep(unsigned millis) {
    ::usleep(millis * 1000);
}

static double x, y, z;
void cb(ConstContactsPtr &_msg)
{
    boost::shared_ptr<const gazebo::msgs::Contacts> contacts = _msg;
    // all the contact return the same values, so only using contact 0 
    if (contacts->contact_size()) {
        if (contacts->contact(0).position_size()) {
            /*
            std::cout << 
                "x=" << contacts->contact(0).position(0).x() << 
                "y=" << contacts->contact(0).position(0).y() << 
                "z=" << contacts->contact(0).position(0).z()  
            << std::endl; 
            //*/
            x = contacts->contact(0).position(0).x();
            y = contacts->contact(0).position(0).y();
            z = contacts->contact(0).position(0).z();  
        }
    }
    return;
}

int main(int _argc, char **_argv)
{
    Gnuplot gp;
    std::cout << "Press Ctrl-C to quit (closing gnuplot window doesn't quit)." << std::endl;

    // Load gazebo
    gazebo::setupClient(_argc, _argv);

    // Create our node for communication
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();

    // Listen to Gazebo world_stats topic
    gazebo::transport::SubscriberPtr sub = node->Subscribe("~/my_robot/leg1/leg1_contact/contacts", cb);


    const int N = 1000;
    //std::vector<double> pts(N);
    std::vector<std::pair<double, double> > pts(N);
    double theta = 0;
    while(1) {
        /*
        for(int i=0; i<N; i++) {
            double alpha = (double(i)/N-0.5) * 10;
            pts[i] = sin(alpha*8.0 + theta) * exp(-alpha*alpha/2.0);
        }
        gp << "plot '-' binary" << gp.binFmt1d(pts, "array") << "with lines notitle\n";
        gp.sendBinary1d(pts);
        gp.flush();
        //*/

        //pts.pop_back();
        pts.push_back(std::make_pair(x, y));
        gp << "plot '-' with points title 'positions'\n";
        gp.send1d(pts);
        gp.flush();

        theta += 0.2;
        gazebo::common::Time::MSleep(10);
        mysleep(100);
    }

    gazebo::shutdown();
    return 0;
}

