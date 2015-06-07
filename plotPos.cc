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

const std::string contact1MsgName = "my_robot::leg1::collision";
const std::string contact2MsgName = "my_robot::leg2::collision";
const std::string contact3MsgName = "my_robot::leg3::collision";
const std::string contact4MsgName = "my_robot::leg4::collision";
const std::string contact5MsgName = "my_robot::leg5::collision";
const std::string contact6MsgName = "my_robot::leg6::collision";

static double x[6], y[6], z[6];

void cb(ConstContactsPtr &_msg)
{
    boost::shared_ptr<const gazebo::msgs::Contacts> contacts = _msg;
    // all the contact return the same values, so only using contact(0) 
    if (contacts->contact_size()) {
        if (contacts->contact(0).position_size()) {
            /*
            std::cout << 
                "x=" << contacts->contact(0).position(0).x() << 
                "y=" << contacts->contact(0).position(0).y() << 
                "z=" << contacts->contact(0).position(0).z()  
            << std::endl; 
            //std::cout << contacts->contact(0).collision1().c_str() << std::endl;
            //*/
            if (contacts->contact(0).collision1().c_str() == contact1MsgName) {
                x[0] = contacts->contact(0).position(0).x();
                y[0] = contacts->contact(0).position(0).y();
                z[0] = contacts->contact(0).position(0).z();  
            } else if (contacts->contact(0).collision1().c_str() == contact2MsgName) {
                x[1] = contacts->contact(0).position(0).x();
                y[1] = contacts->contact(0).position(0).y();
                z[1] = contacts->contact(0).position(0).z();  
            }  else if (contacts->contact(0).collision1().c_str() == contact3MsgName) {
                x[2] = contacts->contact(0).position(0).x();
                y[2] = contacts->contact(0).position(0).y();
                z[2] = contacts->contact(0).position(0).z();  
            }  else if (contacts->contact(0).collision1().c_str() == contact4MsgName) {
                x[3] = contacts->contact(0).position(0).x();
                y[3] = contacts->contact(0).position(0).y();
                z[3] = contacts->contact(0).position(0).z();  
            }  else if (contacts->contact(0).collision1().c_str() == contact5MsgName) {
                x[4] = contacts->contact(0).position(0).x();
                y[4] = contacts->contact(0).position(0).y();
                z[4] = contacts->contact(0).position(0).z();  
            }  else if (contacts->contact(0).collision1().c_str() == contact6MsgName) {
                x[5] = contacts->contact(0).position(0).x();
                y[5] = contacts->contact(0).position(0).y();
                z[5] = contacts->contact(0).position(0).z();  
            } 
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
    gazebo::transport::SubscriberPtr subContact1 = node->Subscribe("~/my_robot/leg1/leg1_contact/contacts", cb);
    gazebo::transport::SubscriberPtr subContact2 = node->Subscribe("~/my_robot/leg2/leg2_contact/contacts", cb);
    gazebo::transport::SubscriberPtr subContact3 = node->Subscribe("~/my_robot/leg3/leg3_contact/contacts", cb);
    gazebo::transport::SubscriberPtr subContact4 = node->Subscribe("~/my_robot/leg4/leg4_contact/contacts", cb);
    gazebo::transport::SubscriberPtr subContact5 = node->Subscribe("~/my_robot/leg5/leg5_contact/contacts", cb);
    gazebo::transport::SubscriberPtr subContact6 = node->Subscribe("~/my_robot/leg6/leg6_contact/contacts", cb);


    const int N = 1000;
    //std::vector<double> pts(N);
    std::vector<std::pair<double, double> > pts135(3);
    std::vector<std::pair<double, double> > pts246(3);
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
           theta += 0.2;
        //*/

        //pts.pop_back();
        for(int i = 1, j = 0; i < 6; i+=2, j++) {
            pts135[j] = std::make_pair(x[i], y[i]);
        }
        for(int i = 0, j = 0; i < 6; i+=2, j++) {
            pts246[j] = std::make_pair(x[i], y[i]);
        }
        //gp << "plot '-' with lines title 'positions' pt 7 ps 5 lt rgb 'blue'\n";

        gp << "set multiplot layout 1,2 rowsfirst\n";
        gp << "plot '-' with linespoints title 'positions' pt 7 ps 5 lt rgb 'blue'\n";
        gp.send1d(pts135);
        gp << "plot '-' with linespoints title 'positions' pt 7 ps 5 lt rgb 'red'\n";
        gp.send1d(pts246);
        gp.flush();

        gazebo::common::Time::MSleep(10);
        mysleep(100);
    }

    gazebo::shutdown();
    return 0;
}

