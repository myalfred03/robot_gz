#ifndef _JOINT_CONTROLLER_GZ_HH_
#define _JOINT_CONTROLLER_GZ_HH_

#include <thread>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "ros/callback_queue.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
//#include <boost/thread/thread.hpp>
double ToG    = 57.295779513;

namespace gazebo
{
  class JointControllerGz : public ModelPlugin
  {
    public: JointControllerGz() {}
    private: std::unique_ptr<ros::NodeHandle> rosNode;
    private: ros::Subscriber rosSub, rosSubPID;
    private: ros::Publisher  rosjoint;
    //private: ros::Subscriber rosSub;
    private: ros::CallbackQueue rosQueue;
    private: std::thread rosQueueThread;
    private: event::ConnectionPtr updateConnection;
    private: physics::ModelPtr model;
    private: physics::JointPtr joints[6];
    private: common::PID pid;
    public:  std::vector<float> jointvalues = std::vector<float>(6);
    public:  double P_, I_, D_;
    public:  std::string jname;
    public: physics::JointPtr joint;//, joint2, joint3, joint4, joint5, joint6 ;
    public: trajectory_msgs::JointTrajectory msg1;

    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      int i;

      if (_model->GetJointCount() == 0)
      {
        std::cerr << "Invalid joint count, JointController plugin not loaded\n";
        return;
      }
      else
      {
        std::cout << "\nThe JointController plugin is attach to model[" << _model->GetName() << "]\n";
      }

      model = _model;
      pid = common::PID(60.0, 4, 10.0);

         joints[0] = this->model->GetJoint("joint_1");// GetJoint("joint_1"); 
         joints[1] = this->model->GetJoint("joint_2"); //GetJoints()[1];//
         joints[2] = this->model->GetJoint("joint_3"); //GetJoints()[1];//
         joints[3] = this->model->GetJoint("joint_4"); //GetJoints()[1];//
         joints[4] = this->model->GetJoint("joint_5"); //GetJoints()[1];//
         joints[5] = this->model->GetJoint("joint_6"); //GetJoints()[1];//

      for(i = 0; i < 6; i++)
      {
        this->model->GetJointController()->SetPositionPID(joints[i]->GetScopedName(), pid);
      }

      if (!ros::isInitialized())
      {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
      }

      rosNode.reset(new ros::NodeHandle("gazebo_client"));
      // ros::SubscribeOptions so =ros::SubscribeOptions::create<trajectory_msgs::JointTrajectory>("/set_joint_trajectory", 
      //                                                                                      1, 
      //                                                                                      boost::bind(&JointControllerGz::SetPosition, this, _1),
      //                                                                                      ros::VoidPtr(), &this->rosQueue);
      
      // rosSub = rosNode->subscribe(so);

	    this->rosSub = this->rosNode->subscribe("/set_joint_trajectory",1, &JointControllerGz::SetPosition, this);
      this->rosSubPID = this->rosNode->subscribe("/pid_value",1, &JointControllerGz::pidupdate, this);
      this->rosjoint = this->rosNode->advertise<trajectory_msgs::JointTrajectory>("joint_values_gazebo", 10);

      rosQueueThread = std::thread(std::bind(&JointControllerGz::QueueThread, this));

         msg1.points.resize(2);
         msg1.points[0].positions.resize(10);
         msg1.points[1].positions.resize(10);

      this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&JointControllerGz::OnUpdate, this, _1));


    }
  public: void OnUpdate(const common::UpdateInfo & _info)
{ 



     for(int i = 0; i < 6; i++)
      {
       model->GetJointController()->SetPositionTarget(joints[i]->GetScopedName(), jointvalues[i]);
       msg1.points[0].positions[i] = joints[i]->Position(0);
       msg1.points[1].positions[i] = joints[i]->GetVelocity(0);
      }
        rosjoint.publish(msg1);
 }


  public: void pidupdate(const trajectory_msgs::JointTrajectory &msg){
         jname = msg.joint_names[0];
   
   //  joint1 = modelo->GetJoint("joint_1");
  // this->joint1 = this->modelo->GetJoints()[1];
         // this->pid1 = common::PID(msg.points[0].positions[0], 10, 1.47, 10, -10,joint1->GetVelocityLimit(0), -1*joint1->GetVelocityLimit(0));
         // this->jointPID->SetPositionPID(this->joint1->GetScopedName(), this->pid1);
       // _union->pid.SetDGain(msg.points[0].positions[2]);
       // _union->pid.SetIGain(msg.points[0].positions[1]);
       // _union->pid.SetPGain(msg.points[0].positions[0]);
       // _union->pid.SetIMax(msg.points[0].positions[3]);
       // _union->pid.SetIMin(msg.points[0].positions[4]);
         P_= msg.points[0].positions[0];
         I_= msg.points[0].positions[1];
         D_= msg.points[0].positions[2];
     pid = common::PID(P_, I_, D_);
     joint = model->GetJoint(jname);
     this->model->GetJointController()->SetPositionPID(joint->GetScopedName(), pid);
          std::cout<<"DATA PID"<<  P_ << I_ << D_ << std::endl;

   }


    public: void SetPosition(const trajectory_msgs::JointTrajectory &msg)
    {

    	       trajectory_msgs::JointTrajectory msgS;
                  int it=0;
                  msgS.points.resize(1);
                  msgS.points[0].positions.resize(6);
                  double jointx, joint;
                  std::vector<double> velvalues(6);

                  std::vector<std::string> jointname(6);
                  it = msg.points.size();

                  for (int m=1; m<it; m++){

                  for (int i = 0; i < 6; i++) {
                    jointvalues[i] = (msg.points[m].positions[i]+0.001)/ToG;
                    velvalues  [i] = msg.points[m].velocities[i];
                    jointname[i] = msg.joint_names[i];
                    std::cout<< jointname[5] << std::endl;


                   //msgpub.points[0].positions[i] = msg.points[0].positions[i];

                  // Comandos::wait(velocities(0));
                //   model->GetJointController()->SetPositionTarget(joints[i]->GetScopedName(), jointvalues[i]);
               //    boost::this_thread::sleep(boost::posix_time::milliseconds(100));

                  }

              } //for points

      // for(int i = 0; i < 6; i++)
      // {
      //   model->GetJointController()->SetPositionTarget(piston[i]->GetScopedName(), msg->data[i]);
      // }
    }

    private: void QueueThread()
    {
      static const double timeout = 0.01;
      while (rosNode->ok())
      {
        rosQueue.callAvailable(ros::WallDuration(timeout));
      }
    }

  };

  GZ_REGISTER_MODEL_PLUGIN(JointControllerGz)
}
#endif