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
            void Load( physics::ModelPtr _parent, sdf::ElementPtr )
            {
                this->model = _parent;
                this->joint1 = _parent->GetJoint("leg1_joint");
                this->joint2 = _parent->GetJoint("leg2_joint");
                this->joint3 = _parent->GetJoint("leg3_joint");
                this->joint4 = _parent->GetJoint("leg4_joint");
                this->joint5 = _parent->GetJoint("leg5_joint");
                this->joint6 = _parent->GetJoint("leg6_joint");
                this->jointBody = _parent->GetJoint("body_joint");
                this->controller = new physics::JointController(model);

                this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                        boost::bind(&ModelPush::OnUpdate, this, _1));

            }

            void OnUpdate( const common::UpdateInfo ) 
            {
                //this->model->SetLinearVel(math::Vector3(1.03, 0, 0));

                //printf("joint1 angle: %d, %d, %d\n", joint1->GetAngle(0), joint1->GetAngle(1), 
                //        joint1->GetAngle(2));
                static bool startLeg = 0;
                std::cout << std::setprecision(2) <<  std::fixed << 
                    "joint1=" << this->joint1->GetAngle(0).Degree() << ", " <<
                    "joint2=" << this->joint2->GetAngle(0).Degree() << ", " <<
                    "joint3=" << this->joint3->GetAngle(0).Degree() << ", " << 
                    "joint4=" << this->joint4->GetAngle(0).Degree() << ", " <<
                    "joint5=" << this->joint5->GetAngle(0).Degree() << ", " << 
                    "joint6=" << this->joint6->GetAngle(0).Degree() << std::endl;

                // get tripod gait
                int velocity = 1; // 1 [m/s] 
                if (this->joint2->GetAngle(0).Degree() > 180) { 
                    this->joint1->SetVelocity(0, velocity); // apply vel to axis 0, 
                    this->joint3->SetVelocity(0, velocity); // it doesn't matter which axis 
                    this->joint5->SetVelocity(0, velocity); // to apply to since it's 1 DOF
                }
                this->joint2->SetVelocity(0, velocity);
                this->joint4->SetVelocity(0, velocity);
                this->joint6->SetVelocity(0, velocity);

                //this->jointBody->SetAngle(0, math::Angle(0));
                myMain();
            }
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

            void myMain()
            {
                return;
            }
    };

    GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}


