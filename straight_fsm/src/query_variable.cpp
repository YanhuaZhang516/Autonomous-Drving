#include <straight_fsm/query_variable.h>


queryVariable::queryVariable()
{
    // ??? queue length
    sub1 = nh.subscribe("VechInfo", 10, &queryVariable::callback1, this);
    sub2 = nh.subscribe("finalObjList/map/objects", 10, &queryVariable::callback2, this);
    proccessData();
}



void queryVariable::callback1(const vaafo_msgs::CarInfo msg){
    //ROS_INFO("Listener: CarInfo, lane_id = %f, twist: %f, pose: %f, acceleration: %f",
    //         msg.lane_id, msg.twist, msg.pose, msg.acceleration);
    //std::cout<<"receiving carinfo...................................."<<std::endl;
    /*std::cout<<"vx: "<<msg.twist.linear.x<<"  vy: "<<msg.twist.linear.x<<std::endl;
    std::cout<<"vx_angle: "<<msg.twist.angular.x<<"  vy_angular: "<<msg.twist.angular.y<<std::endl;
    std::cout<<"pose: ["<<msg.pose.position.x<<", "<<msg.pose.position.y<<", "<<msg.pose.position.z<<"]"<<std::endl;
    std::cout<<"orientation: ["<<msg.pose.orientation.x<<", "<<msg.pose.orientation.y<<", "<<msg.pose.orientation.z<<", "
            <<msg.pose.orientation.w<<"]"<<std::endl;
    std::cout<<"ax: "<<msg.acceleration.linear.x<<"  ay: "<<msg.acceleration.linear.x<<std::endl;
    std::cout<<"s_d: ["<<msg.s_d.x<<", "<<msg.s_d.y<<", "<<msg.s_d.x<<"]"<<std::endl;*/

    egoCar = msg;
}

void queryVariable::callback2(const vaafo_msgs::DetectedObjectArray msg){
    //ROS_INFO("Listener: CarInfo, lane_id = %f, twist: %f, pose: %f, acceleration: %f",
    //         msg.lane_id, msg.twist, msg.pose, msg.acceleration);
    objects = msg.objects;
    int num = objects.size();
    //std::cout<<"receiving DetectedObjectArray...................................."<<std::endl;
    //std::cout<<"num of objects: "<<num<<std::endl;
    //std::cout<<"ID"<<std::endl;
    for(int i=0; i<num; i++){
        //std::cout<<"position: ["<<objects.at(i).pose.position.x<<", "<<objects.at(i).pose.position.y
        //        <<", "<<objects.at(i).pose.position.z<<"]"<<std::endl;
        //std::cout<<"id"<<i<<": "<<objects.at(i).id<<std::endl;
    }
}


// adopting ISO Coordinate System, which is X: front, Y: left, Z: up
int queryVariable::getFront(){
    int n = objects.size();
    float min = 999;
    float distance = -999;
    int index = -1;
    for(int i=0; i<n; i++){
        vaafo_msgs::DetectedObject object = objects.at(i);
        double* result = computeD(object);
        //std::cout<<"d:"<<result[0]<<", cosalpha:"<<result[1]<<std::endl;
        if ((result[0]>-0.1 && result[0]< 0.1)&&
                       result[1]>0){
            distance =  sqrt(pow(objects.at(i).pose.position.x - egoCar.pose.position.x, 2)+
                             pow(objects.at(i).pose.position.y - egoCar.pose.position.y, 2));
            if (distance<min){
                index = i;
                min = distance;
            }
        }
    }
    // return -1 if there are no object detected in front area
    std::cout<<"index of front: "<<index<<std::endl;
    return index;
}

// ??? supose that lane_id count increasedly from left to right, aka from 0 to 1
int queryVariable::getFrontLeft(){
    int n = objects.size();
    float min = 999;
    float distance = -999;
    int index = -1;
    for (int i=0; i<n; i++){
        vaafo_msgs::DetectedObject object = objects.at(i);
        double* result = computeD(object);
        if ((result[0]>-3.6 && result[0]< -3.4)&&
                       result[1]>0){
            distance =  sqrt(pow(objects.at(i).pose.position.x - egoCar.pose.position.x, 2)+
                             pow(objects.at(i).pose.position.y - egoCar.pose.position.y, 2));
            if (distance<min){
                index = i;
                min = distance;
            }
        }
    }
    //std::cout<<"index of front left: "<<index<<std::endl;
    return index;
}

int queryVariable::getFrontRight(){
    int n = objects.size();
    float min = 999;
    float distance = -999;
    int index = -1;
    for (int i=0; i<n; i++){
        vaafo_msgs::DetectedObject object = objects.at(i);
        double* result = computeD(object);
        if ((result[0]>3.4 && result[0]< 3.6)&&
                       result[1]>0){
            distance =  sqrt(pow(objects.at(i).pose.position.x - egoCar.pose.position.x, 2)+
                             pow(objects.at(i).pose.position.y - egoCar.pose.position.y, 2));
            if (distance<min){
                index = i;
                min = distance;
            }
        }
    }
    //std::cout<<"index of front right: "<<index<<std::endl;
    return index;
}

int queryVariable::getBackLeft(){
    int n = objects.size();
    float min = 999;
    float distance = -999;
    int index = -1;
    for (int i=0; i<n; i++){
        vaafo_msgs::DetectedObject object = objects.at(i);
        double* result = computeD(object);
        if ((result[0]>-3.6 && result[0]< -3.4)&&
                       result[1]<0){
            distance =  sqrt(pow(objects.at(i).pose.position.x - egoCar.pose.position.x, 2)+
                             pow(objects.at(i).pose.position.y - egoCar.pose.position.y, 2));
            if (distance<min){
                index = i;
                min = distance;
            }
        }
    }
    //std::cout<<"index of back left:"<<index<<std::endl;
    return index;
}

int queryVariable::getBackRight(){
    int n = objects.size();
    float min = 999;
    float distance = -999;
    int index = -1;
    for (int i=0; i<n; i++){
        vaafo_msgs::DetectedObject object = objects.at(i);
        double* result = computeD(object);
        if ((result[0]>3.4 && result[0]< 3.6)&&
                       result[1]<0){
            distance =  sqrt(pow(objects.at(i).pose.position.x - egoCar.pose.position.x, 2)+
                             pow(objects.at(i).pose.position.y - egoCar.pose.position.y, 2));
            if (distance<min){
                index = i;
                min = distance;
            }
        }
    }
    //std::cout<<"index of back right: "<<index<<std::endl;
    return index;
}

double* queryVariable::computeD(vaafo_msgs::DetectedObject object){
    // test result: if the object runs on the right lane, d=+3.5;
    // if the object runs on the left side, d=-3.5
    double x0 = egoCar.pose.position.x;
    double y0 = egoCar.pose.position.y;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(egoCar.pose.orientation, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    //double theta = pi-yaw;
    double theta = yaw;
    double x2 = object.pose.position.x;
    double y2 = object.pose.position.y;
    double c = y0-tan(theta)*x0;
    double d = (tan(theta)*x2-y2+c)/sqrt(pow(tan(theta), 2)+1);
    double cosalpha = (x2-x0+(y2-y0)*tan(theta)) /
            (sqrt(pow(x2-x0,2)+pow(y2-y0,2)) * sqrt(1+pow(tan(theta), 2)));
    double* result = new double[2];
    result[0] = d;
    result[1] = cosalpha;
    return result;
}

void queryVariable::proccessData(){
    int front = getFront();
    int fLeft = getFrontLeft();
    int fRight = getFrontRight();
    int bLeft = getBackLeft();
    int bRight = getBackRight();
    //vaafo_msgs::DetectedObject nearObjects[] = {f_Object,fl_Object,fr_Object,bl_Object,br_Object};
    int indexArray[] = {front, fLeft, fRight, bLeft, bRight};
    //for(int i=0; i<sizeof(indexArray)/sizeof(indexArray[0]); i++){
    for(int i=0; i<5; i++){
        if(indexArray[i]==-1){
            // no objects nearby
            // ???how to deal with this?
            nearObjects[i].pose.position.x = 9999;
            nearObjects[i].pose.position.y = 9999;
            nearObjects[i].pose.position.z = 1.5;
            nearObjects[i].velocity.linear.x = 999;
            nearObjects[i].velocity.linear.y = 999;
            nearObjects[i].velocity.linear.z = 999;
        }
        else{
            /*nearObjects[i].pose.position.x = objects.at(indexArray[i]).pose.position.x;
            nearObjects[i].pose.position.y = objects.at(indexArray[i]).pose.position.y;
            nearObjects[i].pose.position.z = objects.at(indexArray[i]).pose.position.z;
            nearObjects[i].velocity.linear.x = objects.at(indexArray[i]).velocity.linear.x;
            nearObjects[i].velocity.linear.y = objects.at(indexArray[i]).velocity.linear.y;
            nearObjects[i].velocity.linear.z = objects.at(indexArray[i]).velocity.linear.z;*/
            nearObjects[i] = objects.at(indexArray[i]);
        }
    }
}


