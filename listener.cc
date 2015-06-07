#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo.hh>
#include <iostream>

// Function is called everytime a message is received.
void cb(ConstContactsPtr &_msg)
{
    // Dump the message contents to stdout.
    //std::cout << _msg->DebugString();
    boost::shared_ptr<const gazebo::msgs::Contacts> contacts = _msg;
    if (contacts->contact_size()) {
        if (contacts->contact(0).position_size()) {
            std::cout << 
                "x=" << contacts->contact(0).position(0).x() << 
                "y=" << contacts->contact(0).position(0).y() << 
                "z=" << contacts->contact(0).position(0).z()  
            << std::endl; 
        }
    }
    /*
    for (unsigned int i = 0; i < contacts->contact_size(); ++i)
    {
        for (unsigned int j = 0; j < contacts->contact(i).position_size(); ++j)
        {
            std::cout << contacts->contact(i).position(j).x() << std::endl; 
            (double) contacts->contact(i).position(j).y();
            (double) contacts->contact(i).position(j).z();
            break;
        }
    }
    //*/
    return;
}

int main(int _argc, char **_argv)
{
    // Load gazebo
    gazebo::setupClient(_argc, _argv);

    // Create our node for communication
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();

    // Listen to Gazebo world_stats topic
    gazebo::transport::SubscriberPtr sub = node->Subscribe("~/my_robot/leg1/leg1_contact/contacts", cb);

    // Busy wait loop...replace with your own code as needed.
    while (true)
        gazebo::common::Time::MSleep(10);

    // Make sure to shut everything down.
    gazebo::shutdown();
    return 0;
}
