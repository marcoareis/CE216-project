#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo.hh>

#include <iostream>

/////////////////////////////////////////////////

typedef const boost::shared_ptr<const gazebo::msgs::Vector2d> VectorTwoDPtr;
void cb(VectorTwoDPtr &_msg)
{
    Dump the message contents to stdout.
        std::cout << "x : " <<_msg->x() << "\n";
    std::cout << "y : " <<_msg->y() << "\n";
}

int main(int _argc, char **_argv)
{
    Load gazebo
        gazebo::load(_argc, _argv);

    gazebo::run();

    //Create our node for communication
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();

    Listen to Gazebo world_stats topic
        gazebo::transport::SubscriberPtr sub = node->Subscribe("~/pose_example1", cb);

    // Busy wait loop...replace with your own code as needed.
    while (true)
        gazebo::common::Time::MSleep(10);

    // Make sure to shut everything down.
    gazebo::transport::fini();
}
