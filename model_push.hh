#ifndef MODEL_PUSH_CC
#define MODEL_PUSH_CC

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <gazebo/math/gzmath.hh>


namespace gazebo
{
    class ModelPush : public ModelPlugin
    {
        public:
            void Load( physics::ModelPtr _parent, sdf::ElementPtr );
            /*
            void testFoo(double x, double y, double z) {
                std::cout << "foo: x="<< x << ",y=" << y << ",z=" << z << std::endl;
            }
            //*/
            void OnUpdate( const common::UpdateInfo );
            
        private:

            physics::ModelPtr model;
            physics::JointPtr joint1;
            physics::JointPtr joint2;
            physics::JointPtr joint3;
            physics::JointPtr joint4;
            physics::JointPtr joint5;
            physics::JointPtr joint6;
            physics::JointPtr jointBody;
            physics::JointController * controller;
            event::ConnectionPtr updateConnection;

            void drive135(double, int);
            void drive246(double, int);

            void controller_timer(void);
            void controller_angle1(void);
            void controller_angle2(void);

            transport::NodePtr node;
            gazebo::transport::SubscriberPtr sub;

            
    };

}


#endif
