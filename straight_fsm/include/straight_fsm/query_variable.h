#ifndef QUERY_VARIABLE_H
#define QUERY_VARIABLE_H
#include <ros/ros.h>
#include <vaafo_msgs/CarInfo.h>
#include <vaafo_msgs/DetectedObject.h>
#include <vaafo_msgs/DetectedObjectArray.h>
#include <vaafo_msgs/SensorObjectList.h>
#include <vaafo_msgs/SensorObjectInfo.h>
#include "std_msgs/String.h"
#include <std_msgs/Float32.h>
#include <geometry_msgs/Pose.h>
#include <vector>
#include <tf/tf.h>
#include <math.h>


class queryVariable
{
public:
    queryVariable();
    ~queryVariable(){

    }
    void callback1(const vaafo_msgs::CarInfo msg);
    void callback2(const vaafo_msgs::DetectedObjectArray msg);
    // get the position and speed of the nearby five cars: f,fl,fr,bl,br (in order)
    void proccessData();
    double* computeD(vaafo_msgs::DetectedObject object);
    // required variables for conditions
    int ego_lane_id;
    vaafo_msgs::CarInfo egoCar;
    std::vector<vaafo_msgs::DetectedObject> objects;
    std::vector<vaafo_msgs::SensorObjectInfo> sensorObjects;
    vaafo_msgs::DetectedObject f_Object;
    vaafo_msgs::DetectedObject fl_Object;
    vaafo_msgs::DetectedObject fr_Object;
    vaafo_msgs::DetectedObject bl_Object;
    vaafo_msgs::DetectedObject br_Object;

    ros::NodeHandle nh;
    ros::Subscriber sub1;
    ros::Subscriber sub2;

    int getFrontLeft();
    int getFrontRight();
    int getBackLeft();
    int getFront();
    int getBackRight();
    vaafo_msgs::DetectedObject nearObjects[5];



/*private:
    ros::NodeHandle nh;
    ros::Subscriber sub1;
    ros::Subscriber sub2;
    // get the index of the array from detected object array*/


};
#endif // QUERY_VARIABLE_H


