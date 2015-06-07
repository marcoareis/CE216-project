#include <numeric>
#include "model_push.hh"
#include <gazebo/common/common.hh>
#include <cmath>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo.hh>

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(ModelPush)


void ModelPush::Load( physics::ModelPtr _parent, sdf::ElementPtr )
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
    //* subscribe to messages
    // Create our node for communication
    this->node = transport::NodePtr(new gazebo::transport::Node());
    node->Init("myNode1");

    // Listen to Gazebo world_stats topic
    this->pub = node->Advertise<gazebo::msgs::Vector3d>("~/myCoG");
    //this->pub->WaitForConnection();

    //*/ end
}


void ModelPush::OnUpdate( const common::UpdateInfo ) 
{
    //this->model->SetLinearVel(math::Vector3(1.03, 0, 0));

    //printf("joint1 angle: %d, %d, %d\n", joint1->GetAngle(0), joint1->GetAngle(1), 
    //        joint1->GetAngle(2));
    static bool startLeg = 0;
    /*
       std::cout << std::setprecision(2) <<  std::fixed << 
       "joint1=" << (abs(this->joint1->GetAngle(0).Degree()) % 360) << ", " <<
       "joint2=" << (abs(this->joint2->GetAngle(0).Degree()) % 360) << ", " <<
       "joint3=" << (abs(this->joint3->GetAngle(0).Degree()) % 360) << ", " << 
       "joint4=" << (abs(this->joint4->GetAngle(0).Degree()) % 360) << ", " <<
       "joint5=" << (abs(this->joint5->GetAngle(0).Degree()) % 360) << ", " << 
       "joint6=" << (abs(this->joint6->GetAngle(0).Degree()) % 360) << ", " <<
       "body joint=" << (abs(this->jointBody->GetAngle(0).Degree()) % 360) << std::endl;
    //*/
    // get tripod gait
    double velocity = 1; // 1 [m/s] 
    //this->controller_timer();
    this->controller_angle1();
    //this->controller_angle2();



    // compute center of mass of the entire robot
    this->getCenterOfMass();

    //testFoo();
    //this->model->GetLink("leg1")->
    //this->jointBody->SetAngle(0, math::Angle(0));

    /* experiment with calling contact plugin's public function to get angles.
       static gazebo::ContactPlugin leg1Contact;
       math::Vector3 tmp;
       leg1Contact.gazebo::ContactPlugin::getContactPos(tmp);
       std::cout << "model: " << "x= " << tmp.x << ", y= " << tmp.y  << ", z=" << tmp.z << std::endl;  
    //*/
    return;
}

void ModelPush::getCenterOfMass()
{
    std::vector<gazebo::math::Vector3> allCoG;
    std::vector<double> allMass;
    std::ostringstream ss;
    //*
    // get all mass and cog
    //std::cout << this->model->GetLink("leg1")->GetWorldCoGPose().pos << std::endl;
    for (int i = 0; i < 6; i++) {
        ss << std::string("leg") << (i+1);
        //std::cout << this->model->GetLink(ss.str())->GetInertial()->GetCoG() << std::endl;
        //allCoG.push_back(this->model->GetLink(ss.str())->GetInertial()->GetCoG());
        allCoG.push_back(this->model->GetLink(ss.str())->GetWorldCoGPose().pos);
        allMass.push_back(this->model->GetLink(ss.str())->GetInertial()->GetMass());
        //std::cout << i << ":" << allCoG[i] << std::endl;
        ss.str("");
    }
    
    // dot product of mass and cog
    gazebo::math::Vector3 modelCoG, sum;
    sum = std::inner_product(allCoG.begin(), allCoG.end(), allMass.begin(), gazebo::math::Vector3(0,0,0)); 
    //std::cout << "sum=" << sum << std::endl;
    modelCoG = sum/std::accumulate(allMass.begin(), allMass.end(), 0.0);
    gazebo::msgs::Vector3d msg;
    gazebo::msgs::Set(&msg, modelCoG);
    this->pub->Publish(msg);
    //std::cout << "modelCoG" <<  modelCoG << std::endl;
    //*/
    return;
}

void ModelPush::controller_timer(void)
{
    static int state = 0, stateTransitionCount = 0;
    static common::Time t0;
    int bound = 5;
    int vel_fast = 5;
    int vel_slow = 2.5;
    int angle = 0;
    common::Time now;
    now.SetToWallTime();
    common::Time timeElapsed;
    double period_fast = 1.0;
    double period_slow = 2.3;

    timeElapsed = (now - t0); 
    switch (state) {
        case 0: // START state
            std::cout << "FSM begins" << std::endl;
            t0.SetToWallTime();
            state = 4;
            std::cout << "entering state 4:" << angle << ",count=" << stateTransitionCount <<  std::endl;
            break;
        case 1:
            this->joint2->SetVelocity(0, vel_fast);
            this->joint4->SetVelocity(0, vel_fast);
            this->joint6->SetVelocity(0, vel_fast);
            this->joint1->SetVelocity(0, 0);
            this->joint3->SetVelocity(0, 0);
            this->joint5->SetVelocity(0, 0);
            angle = abs(this->joint2->GetAngle(0).Degree());
            if (timeElapsed.Double() > period_fast) {
                std::cout << "entering state 2(stop245,slow135):" << angle << ",count=" << stateTransitionCount << std::endl;
                std::cout << "time elapsed=" <<  timeElapsed.Double() << std::endl;
                state = 2;
                stateTransitionCount++; 
                t0.SetToWallTime();
            }
            break;
        case 2:
            this->joint2->SetVelocity(0, 0);  
            this->joint4->SetVelocity(0, 0);  
            this->joint6->SetVelocity(0, 0); 
            this->joint1->SetVelocity(0, vel_slow);
            this->joint3->SetVelocity(0, vel_slow);
            this->joint5->SetVelocity(0, vel_slow);
            angle =  abs(this->joint5->GetAngle(0).Degree());
            if (timeElapsed.Double() > period_slow) {
                std::cout << "entering state 3(stop246,fast135):"<< angle << ",count=" << stateTransitionCount <<  std::endl;
                std::cout << "time elapsed=" << timeElapsed.Double() << std::endl;
                t0.SetToWallTime();
                state = 3;
                stateTransitionCount++; 
            }
            break;
        case 3:
            this->joint1->SetVelocity(0, vel_fast);  
            this->joint3->SetVelocity(0, vel_fast);  
            this->joint5->SetVelocity(0, vel_fast); 
            this->joint2->SetVelocity(0, 0);  
            this->joint4->SetVelocity(0, 0);  
            this->joint6->SetVelocity(0, 0); 
            angle =  abs(this->joint5->GetAngle(0).Degree());
        //if ((angle % 180) <= bound) {
        if (timeElapsed.Double() > period_fast) {
            std::cout << "entering state 4(slow246,stop135):" << angle << ",count=" << stateTransitionCount << std::endl;
            state = 4;
            stateTransitionCount++; 
            t0.SetToWallTime();
            std::cout << "time elapsed=" << timeElapsed.Double() << std::endl;
        }
        break;
    case 4:
        this->joint2->SetVelocity(0, vel_slow);  
        this->joint4->SetVelocity(0, vel_slow);  
        this->joint6->SetVelocity(0, vel_slow); 
        this->joint1->SetVelocity(0, 0);
        this->joint3->SetVelocity(0, 0);
        this->joint5->SetVelocity(0, 0);
        angle =  abs(this->joint2->GetAngle(0).Degree());

        //if ((angle % 180) <= bound) {
        if (timeElapsed.Double() > period_slow) {
            std::cout << "entering state 1(fast246,stop135):" << angle << ",count=" << stateTransitionCount <<  std::endl;
            state = 1;
            stateTransitionCount++; 
            t0.SetToWallTime();
            std::cout << "time elapsed=" << timeElapsed.Double() << std::endl;
        }
        break;

} /* end switch */
return;
}

void ModelPush::controller_angle1(void)
{
    static int state = 0, stateTransitionCount = 0;
    static common::Time t0;
    int bound = 5;
    int vel_fast = 15;
    int vel_slow = 7;
    int angle = 0;
    static bool legSet[7] = {false};

    int angle_stand = 130;
    int angle_switch_speed = 220;

    switch (state) {
        case 0: // START state
            std::cout << "FSM begins" << std::endl;
            state = 4;
            std::cout << "entering state 4:" << angle << ",count=" << stateTransitionCount <<  std::endl;
            break;
        case 1:
            this->joint1->SetVelocity(0, 0);
            this->joint3->SetVelocity(0, 0);
            this->joint5->SetVelocity(0, 0);
            if ((abs(this->joint2->GetAngle(0).Degree()) % 360) == angle_stand) {
                this->joint2->SetVelocity(0, 0);  
                legSet[2] = true;
            } else {
                this->joint2->SetVelocity(0, vel_fast);
            }

            if ((abs(this->joint4->GetAngle(0).Degree()) % 360) == angle_stand) {
                this->joint4->SetVelocity(0, 0);  
                legSet[4] = true;
            } else {
                this->joint4->SetVelocity(0, vel_fast);
            }

            if ((abs(this->joint6->GetAngle(0).Degree()) % 360) == angle_stand) {
                this->joint6->SetVelocity(0, 0);  
                legSet[6] = true;
            } else {
                this->joint6->SetVelocity(0, vel_fast);
            }

            if (legSet[2] && legSet[4] && legSet[6]) {
                std::cout << "entering state 2(stop245,slow135):" << ",count=" << stateTransitionCount << std::endl;
                state = 2;
                stateTransitionCount++; 
                legSet[2] = legSet[4] = legSet[6] = false;
            }
            break;
        case 2:
            this->joint2->SetVelocity(0, 0);  
            this->joint4->SetVelocity(0, 0);  
            this->joint6->SetVelocity(0, 0); 
            angle =  abs(this->joint5->GetAngle(0).Degree());
            if ((abs(this->joint1->GetAngle(0).Degree()) % 360) == angle_switch_speed) {
                this->joint1->SetVelocity(0, 0);  
                legSet[1] = true;
            } else {
                this->joint1->SetVelocity(0, vel_slow);
            }
            if ((abs(this->joint3->GetAngle(0).Degree()) % 360) == angle_switch_speed) {
                this->joint3->SetVelocity(0, 0);  
                legSet[3] = true;
            } else {
                this->joint3->SetVelocity(0, vel_slow);
            }

            if ((abs(this->joint5->GetAngle(0).Degree()) % 360) == angle_switch_speed) {
                this->joint5->SetVelocity(0, 0);  
                legSet[5] = true;
            } else {
                this->joint5->SetVelocity(0, vel_slow);
            }

            if (legSet[1] && legSet[3] && legSet[5]) {
                std::cout << "entering state 3(stop246,fast135):"<< angle << ",count=" << stateTransitionCount <<  std::endl;
                state = 3;
                stateTransitionCount++; 
                legSet[1] = legSet[3] = legSet[5] = false;
            }
            break;
        case 3:
            this->joint2->SetVelocity(0, 0);  
            this->joint4->SetVelocity(0, 0);  
            this->joint6->SetVelocity(0, 0); 
            angle =  abs(this->joint5->GetAngle(0).Degree());
            if ((abs(this->joint1->GetAngle(0).Degree()) % 360) == angle_stand) {
                this->joint1->SetVelocity(0, 0);  
                legSet[1] = true;
            } else {
                this->joint1->SetVelocity(0, vel_fast);
            }

            if ((abs(this->joint3->GetAngle(0).Degree()) % 360) == angle_stand) {
                this->joint3->SetVelocity(0, 0);  
                legSet[3] = true;
            } else {
                this->joint3->SetVelocity(0, vel_fast);
            }

            if ((abs(this->joint5->GetAngle(0).Degree()) % 360) == angle_stand) {
                this->joint5->SetVelocity(0, 0);  
                legSet[5] = true;
            } else {
                this->joint5->SetVelocity(0, vel_fast);
            }

            if (legSet[1] && legSet[3] && legSet[5]) {
                std::cout << "entering state 4(slow246,stop135):" << angle << ",count=" << stateTransitionCount << std::endl;
                state = 4;
                stateTransitionCount++; 
                legSet[1] = legSet[3] = legSet[5] = false;
            }
            break;
        case 4:
            this->joint1->SetVelocity(0, 0);
            this->joint3->SetVelocity(0, 0);
            this->joint5->SetVelocity(0, 0);
            angle =  abs(this->joint2->GetAngle(0).Degree());

            if ((abs(this->joint2->GetAngle(0).Degree()) % 360) == angle_switch_speed) {
                this->joint2->SetVelocity(0, 0);  
                legSet[2] = true;
            } else {
                this->joint2->SetVelocity(0, vel_slow);
            }
            if ((abs(this->joint4->GetAngle(0).Degree()) % 360) == angle_switch_speed) {
                this->joint4->SetVelocity(0, 0);  
                legSet[4] = true;
            } else {
                this->joint4->SetVelocity(0, vel_slow);
            }

            if ((abs(this->joint6->GetAngle(0).Degree()) % 360) == angle_switch_speed) {
                this->joint6->SetVelocity(0, 0);  
                legSet[6] = true;
            } else {
                this->joint6->SetVelocity(0, vel_slow);
            }

            if (legSet[2] && legSet[4] && legSet[6]) {
                std::cout << "entering state 1(fast246,stop135):" << angle << ",count=" << stateTransitionCount <<  std::endl;
                state = 1;
                stateTransitionCount++; 
                legSet[2] = legSet[4] = legSet[6] = false;
            }
            break;

    } /* end switch */
    return;
}

static bool legSet[7] = {false};
void ModelPush::drive135(double v, int thetaS)
{
    if ((abs(this->joint1->GetAngle(0).Degree()) % 360) == thetaS) {
        this->joint1->SetVelocity(0, 0);  
        legSet[1] = true;
    } else {
        this->joint1->SetVelocity(0, v);
    }

    if ((abs(this->joint3->GetAngle(0).Degree()) % 360) == thetaS) {
        this->joint3->SetVelocity(0, 0);  
        legSet[3] = true;
    } else {
        this->joint3->SetVelocity(0, v);
    }

    if ((abs(this->joint5->GetAngle(0).Degree()) % 360) == thetaS) {
        this->joint5->SetVelocity(0, 0);  
        legSet[5] = true;
    } else {
        this->joint5->SetVelocity(0, v);
    }
    return;
}

void ModelPush::drive246(double v, int thetaS)
{
    if ((abs(this->joint2->GetAngle(0).Degree()) % 360) == thetaS) {
        this->joint2->SetVelocity(0, 0);  
        legSet[2] = true;
    } else {
        this->joint2->SetVelocity(0, v);
    }

    if ((abs(this->joint4->GetAngle(0).Degree()) % 360) == thetaS) {
        this->joint4->SetVelocity(0, 0);  
        legSet[4] = true;
    } else {
        this->joint4->SetVelocity(0, v);
    }

    if ((abs(this->joint6->GetAngle(0).Degree()) % 360) == thetaS) {
        this->joint6->SetVelocity(0, 0);  
        legSet[6] = true;
    } else {
        this->joint6->SetVelocity(0, v);
    }
    return;
}

void ModelPush::controller_angle2(void)
{
    static int state = 0, stateTransitionCount = 0;
    static common::Time t0;
    int bound = 5;
    int vel_fast = 1.5;//5;
    int vel_slow = .5;//2.5;
    int angle = 0;
    int thetaS = 20;

    switch (state) {
        case 0: // START state
            std::cout << "FSM begins" << std::endl;
            state = 1;
            std::cout << "entering state 1:" << ",count=" << stateTransitionCount <<  std::endl;
            break;
        case 1:
            drive135(vel_fast, thetaS);
            if (legSet[1] && legSet[3] && legSet[5]) {
                drive246(vel_slow, thetaS);
            } else {
                drive246(0, thetaS);
            }

            if (0&&legSet[2] && legSet[4] && legSet[6] && legSet[1] && legSet[3] && legSet[5]) {
                std::cout << "entering state 2(stop245,slow135):" << ",count=" << stateTransitionCount << std::endl;
                //                state = 2;
                stateTransitionCount++; 
                legSet[1] = legSet[3] = legSet[5] = legSet[2] = legSet[4] = legSet[6] = false;
            }
            break;
        case 2:
            drive135(vel_slow, thetaS);
            drive246(vel_slow, thetaS);

            if (legSet[2] && legSet[4] && legSet[6] && legSet[1] && legSet[3] && legSet[5]) {
                std::cout << "entering state 3(stop246,fast135):"<< angle << ",count=" << stateTransitionCount <<  std::endl;
                state = 3;
                stateTransitionCount++; 
                legSet[1] = legSet[3] = legSet[5] = legSet[2] = legSet[4] = legSet[6] = false;
            }
            break;
        case 3:
            this->joint2->SetVelocity(0, 0);  
            this->joint4->SetVelocity(0, 0);  
            this->joint6->SetVelocity(0, 0); 
            angle =  abs(this->joint5->GetAngle(0).Degree());
            if ((abs(this->joint1->GetAngle(0).Degree()) % 360) == 180) {
                this->joint1->SetVelocity(0, 0);  
                legSet[1] = true;
            } else {
                this->joint1->SetVelocity(0, vel_fast);
            }

            if ((abs(this->joint3->GetAngle(0).Degree()) % 360) == 180) {
                this->joint3->SetVelocity(0, 0);  
                legSet[3] = true;
            } else {
                this->joint3->SetVelocity(0, vel_fast);
            }

            if ((abs(this->joint5->GetAngle(0).Degree()) % 360) == 180) {
                this->joint5->SetVelocity(0, 0);  
                legSet[5] = true;
            } else {
                this->joint5->SetVelocity(0, vel_fast);
            }

            if (legSet[1] && legSet[3] && legSet[5]) {
                std::cout << "entering state 4(slow246,stop135):" << angle << ",count=" << stateTransitionCount << std::endl;
                state = 4;
                stateTransitionCount++; 
                legSet[1] = legSet[3] = legSet[5] = false;
            }
            break;
        case 4:
            this->joint1->SetVelocity(0, 0);
            this->joint3->SetVelocity(0, 0);
            this->joint5->SetVelocity(0, 0);
            angle =  abs(this->joint2->GetAngle(0).Degree());

            if ((abs(this->joint2->GetAngle(0).Degree()) % 360) == 359) {
                this->joint2->SetVelocity(0, 0);  
                legSet[2] = true;
            } else {
                this->joint2->SetVelocity(0, vel_slow);
            }
            if ((abs(this->joint4->GetAngle(0).Degree()) % 360) == 359) {
                this->joint4->SetVelocity(0, 0);  
                legSet[4] = true;
            } else {
                this->joint4->SetVelocity(0, vel_slow);
            }

            if ((abs(this->joint6->GetAngle(0).Degree()) % 360) == 359) {
                this->joint6->SetVelocity(0, 0);  
                legSet[6] = true;
            } else {
                this->joint6->SetVelocity(0, vel_slow);
            }

            if (legSet[2] && legSet[4] && legSet[6]) {
                std::cout << "entering state 1(fast246,stop135):" << angle << ",count=" << stateTransitionCount <<  std::endl;
                state = 1;
                stateTransitionCount++; 
                legSet[2] = legSet[4] = legSet[6] = false;
            }
            break;

    } /* end switch */
    return;
}
