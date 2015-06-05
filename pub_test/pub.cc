#include <gazebo/gazebo.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <math/gzmath.hh>

#include <iostream>


int main(int _argc, char **_argv)
{
    Load gazebo
        gazebo::load(_argc, _argv);

    // Create our node for communication
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();

    // Start transport
    gazebo::transport::run();

    // Publish to a Gazebo topic
    gazebo::transport::PublisherPtr pub =
        node->Advertise<gazebo::msgs::Vector2d>("~/pose_example1");

    // Wait for a subscriber to connect
    pub->WaitForConnection();

    // Publisher loop...replace with your own code.
    while (true)
    {
        gazebo::common::Time::MSleep(100);
        gazebo::math::Vector2d vect(5, 7);
        gazebo::msgs::Vector2d msg;
        gazebo::msgs::Set(&msg, vect);
        pub->Publish(msg);
    }

    // Make sure to shut everything down.
    gazebo::transport::fini();
}
