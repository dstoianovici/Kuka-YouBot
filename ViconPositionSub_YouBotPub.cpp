/*

By: Dan Stoianovici & Troy Southard
Spring Quarter 2018

Mechatronics II: A Kuka Project

This code impliments ROS commands and YouBot Driver package controls
in order to perfrom a pick and place task from positions obtained
from a Vicon tracker camera system.

*/

#include "ros/ros.h"
#include "youbot_arm_kinematics/inverse_kinematics.h"

#include "stdio.h"
#include "math.h"
#include "iostream"
#include "std_msgs/String.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/Twist.h"
#include "brics_actuator/JointPositions.h"
#include "tf2_msgs/TFMessage.h"
#include "boost/units/systems/si.hpp"
#include "boost/units/io.hpp"
#include "trajectory_msgs/JointTrajectory.h"

#define linear_time  12
#define angular_vel 0
#define armRadius 0.55
#define gripperTolerance 0.1
#define GrippyBoi 0

ros::Publisher platformPublisher;
ros::Publisher armPublisher;
ros::Publisher gripperPublisher;

//brics_actuator::JointPositions Arm_msg;
//brics_actuator::JointPositions Gripper_msg;
//geometry_msgs::Twist youbot_msg;

static const int numberOfArmJoints = 5;
static const int numberOfGripperJoints = 2;

float tbox_x;  // x&y positions from vicon daddy
float tbox_y;
float KuukBase_x;
float KuukBase_y;
float KuukGrip_x;
float KuukGrip_y;
float GripRightFing_x;
float GripRightFing_y;


float GripDist_x; // variables to calculate distances between the points
float GripDist_y;
float BaseDist_x;
float BaseDist_y;
float FingDist_x;
float FingDist_y;

////Action Flag
//bool WeOutHereBaby = false;

//Call Back functions for subscribers
void positions_TestBox(const geometry_msgs::TransformStamped& msg){
    geometry_msgs::Vector3 coord = msg.transform.translation;
    tbox_x = coord.x;
    tbox_y = coord.y;
}

void positions_KukaBase(const geometry_msgs::TransformStamped& msg){
    geometry_msgs::Vector3 coord = msg.transform.translation;
    KuukBase_x = coord.x;
    KuukBase_y = coord.y;

    BaseDist_x = KuukBase_x - tbox_x;
    BaseDist_y = KuukBase_y - tbox_y;

    ROS_INFO("Distance to obj: (%f, %f)", BaseDist_x, BaseDist_y);
}

void positions_KukaGripper(const geometry_msgs::TransformStamped& msg){
    geometry_msgs::Vector3 coord = msg.transform.translation;
    KuukGrip_x = coord.x;
    KuukGrip_y = coord.y;

    GripDist_x = KuukGrip_x - tbox_x;
    GripDist_y = KuukGrip_y - tbox_y;

    ROS_INFO("Distance from gripper to obj: (%f, %f)", GripDist_x, GripDist_y);
}

//void positions_GripRightFinger(const geometry_msgs::TransformStamped& msg){
//    geometry_msgs::Vector3 coord = msg.transform.translation;
//    GripRightFing_x = coord.x;
//    GripRightFing_y = coord.y;

//    FingDist_x = GripRightFing_x - tbox_x;
//    FingDist_y = GripRightFing_y - tbox_y;

//    ROS_INFO("Distance gripper to obj: (%f, %f)", GripDist_x, GripDist_y);
//}

// create a brics actuator message with the given joint position values
brics_actuator::JointPositions createArmPositionCommand(std::vector<double>& newPositions) {
        int numberOfJoints = 5;
        brics_actuator::JointPositions Arm_msg;

        if (newPositions.size() < numberOfJoints)
                return Arm_msg; // return empty message if not enough values provided

        for (int i = 0; i < numberOfJoints; i++) {
                // Set all values for one joint, i.e. time, name, value and unit
                brics_actuator::JointValue joint;
                joint.timeStamp = ros::Time::now();
                joint.value = newPositions[i];
                joint.unit = boost::units::to_string(boost::units::si::radian);

                // create joint names: "arm_joint_1" to "arm_joint_5" (for 5 DoF)
                std::stringstream jointName;
                jointName << "arm_joint_" << (i + 1);
                joint.joint_uri = jointName.str();

                // add joint to message
                Arm_msg.positions.push_back(joint);
        }
        return Arm_msg;
}

// create a brics actuator message for the gripper using the same position for both fingers
brics_actuator::JointPositions createGripperPositionCommand(double newPosition) {
        brics_actuator::JointPositions Arm_msg;

        brics_actuator::JointValue joint;
        joint.timeStamp = ros::Time::now();
        joint.unit = boost::units::to_string(boost::units::si::meter); // = "m"
        joint.value = newPosition;
        joint.joint_uri = "gripper_finger_joint_l";
        Arm_msg.positions.push_back(joint);
        joint.joint_uri = "gripper_finger_joint_r";
        Arm_msg.positions.push_back(joint);

        return Arm_msg;
}

// move arm
void moveArm_Pick() {
        brics_actuator::JointPositions Arm_msg;

        std::vector<double> jointvalues(5);

        // move to pick position
        jointvalues[0] = 2.94;
        jointvalues[1] = 2.61;
        jointvalues[2] = -1.42;
        jointvalues[3] = 0.85;
        jointvalues[4] = 2.92;
        Arm_msg = createArmPositionCommand(jointvalues);
        armPublisher.publish(Arm_msg);
        //ros::spinOnce();
        return;
}

void moveArm_Viper() {
        brics_actuator::JointPositions Arm_msg;

        std::vector<double> jointvalues(5);

        // move to viper position
        jointvalues[0] = 2.91;
        jointvalues[1] = 1.59;
        jointvalues[2] = -1.63;
        jointvalues[3] = 1.98;
        jointvalues[4] = 2.95;
        Arm_msg = createArmPositionCommand(jointvalues);
        armPublisher.publish(Arm_msg);
        //ros::spinOnce();
        return;
}

void moveArm_Home() {
        brics_actuator::JointPositions Arm_msg;

        std::vector<double> jointvalues(5);

        // move to home position
        jointvalues[0] = 0.11;
        jointvalues[1] = 0.11;
        jointvalues[2] = -0.11;
        jointvalues[3] = .11;
        jointvalues[4] = .12;
        Arm_msg = createArmPositionCommand(jointvalues);
        armPublisher.publish(Arm_msg);
        //ros::spinOnce();
        return;
}

void moveArm_Candle() {
        brics_actuator::JointPositions Arm_msg;

        std::vector<double> jointvalues(5);

        // move to candle position
        jointvalues[0] = 2.95;
        jointvalues[1] = 1.05;
        jointvalues[2] = -2.44;
        jointvalues[3] = 1.73;
        jointvalues[4] = 2.95;
        Arm_msg = createArmPositionCommand(jointvalues);
        armPublisher.publish(Arm_msg);
        //ros::spinOnce();
        return;
}

// open and close gripper
void moveGripper_Open() {
        brics_actuator::JointPositions Gripper_msg;

        // open gripper
        Gripper_msg = createGripperPositionCommand(0.011);
        gripperPublisher.publish(Gripper_msg);
        //ros::spinOnce();
        return;
}

void moveGripper_CloseOnObj() {
        brics_actuator::JointPositions Gripper_msg;

        // close gripper
        Gripper_msg = createGripperPositionCommand(GrippyBoi);
        gripperPublisher.publish(Gripper_msg);
        //ros::spinOnce();
        return;
}

void moveGripper_CloseAll() {
        brics_actuator::JointPositions Gripper_msg;
        // close gripper
        Gripper_msg = createGripperPositionCommand(0);
        gripperPublisher.publish(Gripper_msg);
        //ros::spinOnce();
        return;
}

//Move Platform
void movePlatform_FullStop(){
    geometry_msgs::Twist youbot_msg;

    youbot_msg.linear.x =0;
    youbot_msg.linear.y =0;
    youbot_msg.linear.z = 0;     //No angular velocity or z direction velocity for now
    youbot_msg.angular.x = 0;
    youbot_msg.angular.y = 0;
    youbot_msg.angular.z = 0;
    platformPublisher.publish(youbot_msg);
    //ros::spinOnce();
    return;
}

void movePlatform_CCWSpin(){
    geometry_msgs::Twist youbot_msg;

    youbot_msg.linear.x = 0; //stop base from moving linearly
    youbot_msg.linear.y = 0;
    youbot_msg.linear.z = 0;
    youbot_msg.angular.x = 0;
    youbot_msg.angular.y = 0;
    youbot_msg.angular.z = 0.3; //Rotate counter clockwise
    platformPublisher.publish(youbot_msg);
    //ros::spinOnce();
    return;
}

void movePlatform_CWSpin(){
    geometry_msgs::Twist youbot_msg;

    youbot_msg.linear.x = 0; //stop base from moving linearly
    youbot_msg.linear.y = 0;
    youbot_msg.linear.z = 0;
    youbot_msg.angular.x = 0;
    youbot_msg.angular.y = 0;
    youbot_msg.angular.z = -0.3; //Rotate clockwise
    platformPublisher.publish(youbot_msg);
    //ros::spinOnce();
    return;
}

void movePlatform_Lin2Point(float DistX, float DistY){
    geometry_msgs::Twist youbot_msg;

    youbot_msg.linear.x = DistX / linear_time;
    youbot_msg.linear.y = DistY / linear_time;
    youbot_msg.linear.z = 0;
    youbot_msg.angular.x = 0;
    youbot_msg.angular.y = 0;
    youbot_msg.angular.z = 0;
    platformPublisher.publish(youbot_msg);
    //ros::spinOnce();
    return;
}
void movePlatform_BackWardsBit(){
    geometry_msgs::Twist youbot_msg;

    youbot_msg.linear.x = -.1;
    youbot_msg.linear.y = 0;
    youbot_msg.linear.z = 0;
    youbot_msg.angular.x = 0;
    youbot_msg.angular.y = 0;
    youbot_msg.angular.z = 0;
    platformPublisher.publish(youbot_msg);
    ros::Duration(5).sleep();
    youbot_msg.linear.x = 0;
    youbot_msg.linear.y = 0;
    youbot_msg.linear.z = 0;
    youbot_msg.angular.x = 0;
    youbot_msg.angular.y = 0;
    youbot_msg.angular.z = 0;
    platformPublisher.publish(youbot_msg);

    //ros::spinOnce();
    return;
}

void movePlatform_Home(float currX, float currY){

    geometry_msgs::Twist youbot_msg;

    youbot_msg.linear.x = (currX - 0) / linear_time;
    youbot_msg.linear.y = (currY - 0) / linear_time;
    youbot_msg.linear.z = 0;
    youbot_msg.angular.x = 0;
    youbot_msg.angular.y = 0;
    youbot_msg.angular.z = 0;
    platformPublisher.publish(youbot_msg);
    //ros::spinOnce();
    return;
};

//Quick Maffs
float distCalc(float distx,float disty){
    float DistMag = sqrt(pow(distx,2.0) + pow(disty,2.0));
    return DistMag;
}



int main(int argc, char **argv)
{
    //ROS configuration
    ros::init(argc, argv, "ViconIn_DriverOut");
    ros::NodeHandle nh;

    //Subscriber Initialization
    ros::Subscriber sub = nh.subscribe("/vicon/TestBox/TestBox", 100, positions_TestBox);
    ros::Subscriber sub1 = nh.subscribe("/vicon/KukaBase/KukaBase", 100, positions_KukaBase);
    ros::Subscriber sub2 = nh.subscribe("/vicon/KukaGripper/KukaGripper", 100, positions_KukaGripper);
    //ros::Subscriber sub3 = nh.subscribe("/vicon/GripRightFinger/GripRightFinger", 100, positions_GripRightFinger);


    //Publisher Initialization
    platformPublisher = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    armPublisher = nh.advertise<brics_actuator::JointPositions>("arm_1/arm_controller/position_command", 10);
    gripperPublisher = nh.advertise<brics_actuator::JointPositions>("arm_1/gripper_controller/position_command", 10);
    ROS_INFO("TICK TOCK"); // Sleep indicator to show that the system is waiting until all configurations have been initialized
    ros::Duration(5).sleep();

    //System is setup and all is clear
    ROS_INFO("System is setup and all is clear");

    //Return System to Home Position
    movePlatform_FullStop();
    moveArm_Home();
    moveGripper_CloseAll();
    ros::Duration(5).sleep();
    ros::spinOnce();
    ROS_INFO("YouBot is Home");

 //First Search Phase; moves the base until the arm is within working range////////////////////////////////

    while(distCalc(BaseDist_x,BaseDist_y) > armRadius){
        movePlatform_Lin2Point(BaseDist_x,BaseDist_y);
        ros::spinOnce();
    }

    //Turns off linear motion when body is in range, prepare arm to search for object in 2nd phase
    movePlatform_FullStop();
    moveGripper_Open();
    moveArm_Viper();
    ROS_INFO("YouBot is within arm range...napping");
    ros::Duration(3).sleep();
    ROS_INFO("It was a good nap");


//Second Search Phase; begin searching for object with gripper////////////////////////////////

    while(distCalc(GripDist_x, GripDist_y) > gripperTolerance){
        if(GripDist_y - tbox_y > 0){ // If TestBox is to the left of  gripper
            movePlatform_CCWSpin();
            ROS_INFO("YouBot is searching for object with arm CCW");
            ros::spinOnce();
        }

        else if(GripDist_y - tbox_y < 0){ // If TestBox is to the right of  gripper
            movePlatform_CWSpin();
            ROS_INFO("YouBot is searching for object with arm CW");
            ros::spinOnce();
        }
    }


//Third Search Phase; arm is oriented correctly, back up and begin precise search of TestBox with gripper////////////////////////////////
    ros::Duration(1).sleep();
    movePlatform_BackWardsBit();
    moveArm_Pick();
    ros::Duration(1).sleep();

    while(distCalc(GripDist_x,GripDist_y) > 0.02){  // Holonomically moves base towards TestBox until accuracy of 1cm is reached
        movePlatform_Lin2Point((GripDist_x),GripDist_y);
        ROS_INFO("SUH DUDE");
        ros::spinOnce();
    }

//Pick Phase; The gripper is over object close the maw.

    //Gripper is in position over object, the while loop has exited
    ROS_INFO("Gripper is in position");
    movePlatform_FullStop();
    ros::Duration(2).sleep();

    ros::Duration(3).sleep();
    moveGripper_CloseOnObj();
    ros::Duration(2).sleep();
    moveArm_Candle(); //Triumphantly lifts TextBox above its head.
    ros::Duration(3).sleep();
    movePlatform_Home(KuukBase_x, KuukBase_y);



    return 0;
}

