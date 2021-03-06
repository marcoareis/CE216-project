#include <vector>
#include <cmath>
#include <boost/tuple/tuple.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo.hh>
#include <iostream>
#include <algorithm>
#include "gnuplot-iostream.h"

#include <unistd.h>
inline void mysleep(unsigned millis) {
    ::usleep(millis * 1000);
}

struct Point
{
    double x;
    double y;
};
bool isInside(Point polygon[], int n, Point p);
bool PointInPolygon(gazebo::math::Vector3 point, std::vector<Point> polygon);

const std::string contact1MsgName = "my_robot::leg1::collision";
const std::string contact2MsgName = "my_robot::leg2::collision";
const std::string contact3MsgName = "my_robot::leg3::collision";
const std::string contact4MsgName = "my_robot::leg4::collision";
const std::string contact5MsgName = "my_robot::leg5::collision";
const std::string contact6MsgName = "my_robot::leg6::collision";

//static double x[6], y[6], z[6];
std::vector<double> x(6), y(6), z(6);
std::vector<gazebo::math::Vector3 > legPts(6);
gazebo::math::Vector3 cog;
int legSet = 1; // 1=135, 2=246

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
                legPts[0].x = contacts->contact(0).position(0).x();
                legPts[0].y = contacts->contact(0).position(0).y();
                legPts[0].z = contacts->contact(0).position(0).z();  
                legSet = 1;
            } else if (contacts->contact(0).collision1().c_str() == contact2MsgName) {
                legPts[1].x = contacts->contact(0).position(0).x();
                legPts[1].y = contacts->contact(0).position(0).y();
                legPts[1].z = contacts->contact(0).position(0).z();  
                legSet = 2;
            }  else if (contacts->contact(0).collision1().c_str() == contact3MsgName) {
                legPts[2].x = contacts->contact(0).position(0).x();
                legPts[2].y = contacts->contact(0).position(0).y();
                legPts[2].z = contacts->contact(0).position(0).z();  
                legSet = 1;
            }  else if (contacts->contact(0).collision1().c_str() == contact4MsgName) {
                legPts[3].x = contacts->contact(0).position(0).x();
                legPts[3].y = contacts->contact(0).position(0).y();
                legPts[3].z = contacts->contact(0).position(0).z();  
                legSet = 2;
            }  else if (contacts->contact(0).collision1().c_str() == contact5MsgName) {
                legPts[4].x = contacts->contact(0).position(0).x();
                legPts[4].y = contacts->contact(0).position(0).y();
                legPts[4].z = contacts->contact(0).position(0).z();  
                legSet = 1;
            }  else if (contacts->contact(0).collision1().c_str() == contact6MsgName) {
                legPts[5].x = contacts->contact(0).position(0).x();
                legPts[5].y = contacts->contact(0).position(0).y();
                legPts[5].z = contacts->contact(0).position(0).z();  
                legSet = 2;
            } 
        }
    }
    return;
}

void cb_CoG(ConstVector3dPtr &_msg)
{
    cog.x = _msg->x();
    cog.y = _msg->y();
    cog.z = _msg->z();
    return;
}

double getMinDist2Triangle(void)
{
    std::vector<double> dists;
    switch (legSet) {
        case 1:
            dists.push_back(cog.GetDistToLine(legPts[0], legPts[4]));
            dists.push_back(cog.GetDistToLine(legPts[0], legPts[2]));
            dists.push_back(cog.GetDistToLine(legPts[2], legPts[4]));
            break;
        case 2:
            dists.push_back(cog.GetDistToLine(legPts[1], legPts[5]));
            dists.push_back(cog.GetDistToLine(legPts[1], legPts[3]));
            dists.push_back(cog.GetDistToLine(legPts[3], legPts[5]));
            break;
    }

    return *std::min_element(dists.begin(), dists.end());
}


int main(int _argc, char **_argv)
{
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
    gazebo::transport::SubscriberPtr subCoG = node->Subscribe("/gazebo/myNode1/myCoG", cb_CoG);



    gazebo::transport::PublisherPtr reset_pub;

    reset_pub = node->Advertise<gazebo::msgs::WorldControl>("~/world_control");

    gazebo::msgs::WorldControl w_ctrl;


    //std::vector<double> pts(N);
    std::vector<std::pair<double, double> > pts135(5);
    std::vector<std::pair<double, double> > pts246(5);
    std::vector<std::pair<double, double> > ptCoG(1);
    std::vector<double> minDists;
/*
    for (int i = 0; i < 3; i++) {
    */
    Gnuplot gp;
    int instableCount = 0;
    while(1) {
        for(int i = 1, j = 0; i < 6; i+=2, j++) {
            pts246[j] = std::make_pair(legPts[i].x, legPts[i].y);
        }
        pts246[3] = std::make_pair(legPts[1].x, legPts[1].y); // add initial point to connect the last to first
        for(int i = 0, j = 0; i < 6; i+=2, j++) {
            pts135[j] = std::make_pair(legPts[i].x, legPts[i].y);
        }
        pts135[3] = std::make_pair(legPts[0].x, legPts[0].y); // add initial point to connect the last to first
        //ptCoG[0] = std::make_pair(cog.x, cog.y);

        // add the center of mass to plot on the same grpah
        pts135[4] = std::make_pair(cog.x, cog.y);
        pts246[4] = std::make_pair(cog.x, cog.y);

        minDists.push_back(getMinDist2Triangle());

        gp << "set multiplot layout 2,1 rowsfirst\n";
        gp << "set nokey\n";

        Point cog_check;
        cog_check.x = cog.x;
        cog_check.y = cog.y;
        Point polygon[3];
        switch (legSet) {
            case 1:
                polygon[0].x = pts135[0].first; polygon[0].y = pts135[0].second;
                polygon[1].x = pts135[1].first; polygon[1].y = pts135[1].second;
                polygon[2].x = pts135[2].first; polygon[2].y = pts135[2].second;
                gp << "plot '-' with linespoints title 'leg 135' pt 7 ps 5 lt rgb 'blue'\n";
                gp.send1d(pts135);
                break;
            case 2:
                polygon[0].x = pts246[0].first; polygon[0].y = pts246[0].second;
                polygon[1].x = pts246[1].first; polygon[1].y = pts246[1].second;
                polygon[2].x = pts246[2].first; polygon[2].y = pts246[2].second;
                gp << "plot '-' with linespoints title 'leg 246' pt 7 ps 5 lt rgb 'red'\n";
                gp.send1d(pts246);
                break;
        }
        //gp << "plot '-' with points title 'cog' pt 7 ps 5 lt rgb 'black'\n";
        //gp.send1d(ptCoG);

        std::vector<Point> vecPolygon(polygon, polygon + sizeof(polygon)/sizeof(polygon[0]));

        if (!PointInPolygon(cog, vecPolygon)) { 
            minDists.back() = -minDists.back(); 
            instableCount++;
        }
        gp << "plot '-' binary" << gp.binFmt1d(minDists, "array") << "with points pt 7 ps 1 notitle\n";
        gp.sendBinary1d(minDists);
        if (minDists.size() > 200) {
            break;
        }
        if (minDists.size() > 1000) {
            minDists.erase(minDists.begin());
        }
        gp << "unset multiplot\n";
        gp.flush();

        gazebo::common::Time::MSleep(10);
        mysleep(50);
    } // while() 

    std::cout << "instable points= " << instableCount << std::endl;
    /*
       gp << std::endl;
       w_ctrl.mutable_reset()->set_all(true); //I believe this is equal to indicating a world reset
       reset_pub->Publish(w_ctrl);
       } // for
       */
    gazebo::shutdown();
    return 0;
}

bool PointInPolygon(gazebo::math::Vector3 point, std::vector<Point> points)
{
    int i, j, nvert = points.size();
    bool c = false;

    for(i = 0, j = nvert - 1; i < nvert; j = i++) {
        if( ( (points[i].y >= point.y ) != (points[j].y >= point.y) ) &&
                (point.x <= (points[j].x - points[i].x) * (point.y - points[i].y) / (points[j].y - points[i].y) + points[i].x)
          )
            c = !c;
    }

    return c;
}
