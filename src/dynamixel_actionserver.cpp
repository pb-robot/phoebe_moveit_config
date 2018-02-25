/*
   Cyton Manipulator Action Server
   This server will listen for FollowJointTrajectoryAction and execute the goal on hardware

   Created by Gert Kanter <gert@cs.ttu.ee>
   Tallinn University of Technology
 */

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/server/simple_action_server.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <dynamixel_controllers/SetSpeed.h>

#define SERVICE_TIMEOUT 5

typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> Server;

class DynamixelServo
{
   private:
      std::string name_;
      ros::ServiceClient service_;
      ros::Publisher publisher_;

   public:
      DynamixelServo(ros::NodeHandle n, std::string name) :
         name_(name),
         service_(n.serviceClient<dynamixel_controllers::SetSpeed>(name + "/set_speed")),
         publisher_(n.advertise<std_msgs::Float64>(name + "/command", 1000, false))
   { }
      std::string getName()
      {
         return name_;
      }
      ros::ServiceClient getService()
      {
         return service_;
      }
      ros::Publisher getPublisher()
      {
         return publisher_;
      }
      void serviceCall(dynamixel_controllers::SetSpeed call)
      {
         if (!service_.call(call))
         {
            ROS_ERROR("Failed to call service %s", service_.getService().c_str());
         }
      }
      void publish(std_msgs::Float64 message)
      {
         publisher_.publish(message);
      }
};

class Controller
{
   private:
      std::vector<DynamixelServo> servos_;
      std::vector<int> joint_map_;

   public:
      Controller(ros::NodeHandle n, std::string names[], int total) 
      {
         for (int i = 0; i < total; ++i)
         {
            joint_map_.push_back(-1); // -1 means it is not mapped
            servos_.push_back(DynamixelServo(n, names[i]));
            if (!servos_[i].getService().waitForExistence(ros::Duration(SERVICE_TIMEOUT))) ROS_ERROR("%s service is unavailable!", servos_[i].getService().getService().c_str());
         }
      }

      void jointMap(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal)
      {
         // create the FollowJointTrajectory message joint_names array correspondence map (i.e. right_arm_elbow_pitch -> [0], right_arm_elbow_roll_joint -> [1] etc.) 
         for (int i = 0; i < goal->trajectory.joint_names.size(); ++i)
         {
            for (int j = 0; j < servos_.size(); ++j)
               if (goal->trajectory.joint_names[i] == servos_[j].getName())
                  joint_map_[i] = j;
            if (joint_map_[i] == -1) ROS_ERROR("Unknown joint name in FollowJointTrajectory goal message!");
         }
      }

      void execute(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal, Server* as)
      {
         ROS_INFO("Executing trajectory...");

         jointMap(goal);

         dynamixel_controllers::SetSpeed speed;
         std_msgs::Float64 angle;
         ROS_ERROR("Trajecory size = %lu", goal->trajectory.points.size());
         for (int i = 0; i < goal->trajectory.points.size(); ++i)
         {
            ROS_INFO("Trajectory point [%d]", i);
            ROS_INFO("---------------------");
            ROS_ERROR("Positions size = %lu", goal->trajectory.points[i].positions.size());
            for (int j = 0; j < goal->trajectory.points[i].positions.size(); ++j)
            {
               //ROS_INFO("[%d] %.3f rad/s, %.3f rad", j, goal->trajectory.points[i].velocities[j], goal->trajectory.points[i].positions[j]);
               if (goal->trajectory.points[i].velocities.size() > 0)
               {
                  speed.request.speed = abs(goal->trajectory.points[i].velocities[j]); // is abs correct?
               }
               else
               {
                  // Velocities are not defined, we have to calculate them
                  if (i == 0)
                  {
                     speed.request.speed = (goal->trajectory.points[i+1].positions[j] - goal->trajectory.points[i].positions[j]) / ros::Duration(goal->trajectory.points[i+1].time_from_start - goal->trajectory.points[i].time_from_start).toSec();
                  }
                  else
                  {
                     speed.request.speed = (goal->trajectory.points[i].positions[j] - goal->trajectory.points[i-1].positions[j]) / ros::Duration(goal->trajectory.points[i].time_from_start - goal->trajectory.points[i-1].time_from_start).toSec();
                  }
                  if (abs(speed.request.speed) > 0.2) // MAX SPEED 0.2 RAD/S
                     speed.request.speed = 0.2;
                  ROS_INFO("Calculated speed == %.3f", speed.request.speed);


                  speed.request.speed = 0.05;
               }
               angle.data = goal->trajectory.points[i].positions[j];
               if (joint_map_[j] >= 0)
               {
                  servos_[joint_map_[j]].serviceCall(speed);
                  servos_[joint_map_[j]].publish(angle);
               }
            }
            if (i+1 < goal->trajectory.points.size()) ros::Duration(goal->trajectory.points[i+1].time_from_start - goal->trajectory.points[i].time_from_start).sleep();
         }

         as->setSucceeded();
      }
};


int main(int argc, char** argv)
{
   ros::init(argc, argv, "dynamixel_actionserver");
   ros::NodeHandle n;
   ROS_INFO("Setting up dynamixel servo controllers...");
   //std::string names[8] = {"right_arm_elbow_pitch_joint", "right_arm_elbow_roll_joint", "right_arm_shoulder_pitch_joint", "right_arm_shoulder_roll_joint", "right_arm_wrist_pitch_joint", "right_arm_wrist_roll_joint", "right_arm_wrist_yaw_joint", "right_arm_gripper_joint"};
   std::string names[7] = {"right_arm_shoulder_roll_joint", "right_arm_shoulder_pitch_joint", "right_arm_shoulder_yaw_joint", "right_arm_elbow_pitch_joint", "right_arm_elbow_yaw_joint", "right_arm_wrist_pitch_joint", "right_arm_wrist_roll_joint"};
   Controller controller(n, names, 7);
   ROS_INFO("Starting dynamixel action server...");
   Server server(n, "right_arm_controller/follow_joint_trajectory", boost::bind(&Controller::execute, controller, _1, &server), false);
   server.start();
   ROS_INFO("Action server running!");
   ros::spin();
   return 0;
}
