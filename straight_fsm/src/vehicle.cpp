#include <straight_fsm/vehicle.h>



float fsm_state::t_LastFaliure = -1;
bool fsm_state::turnLeft_Motiv = false;
bool fsm_state::turnRight_Motiv = false;
clock_t fsm_state::startTime_w = -1;
clock_t fsm_state::startTime_f = -1;
float fsm_state::t_wait = -1;

double* getVelocity(geometry_msgs::Twist velocity, geometry_msgs::Quaternion quat_msg){
    double vx = velocity.linear.x;
    double vy = velocity.linear.y;
    tf::Quaternion quat_tf;
    tf::quaternionMsgToTF(quat_msg, quat_tf);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat_tf).getRPY(roll, pitch, yaw);
    double vs = vx * cos(yaw) + vy * sin(yaw);
    double vd = vx * sin(yaw) - vy * cos(yaw);
    double *vel = new double[2];
    vel[0] = vs;
    vel[1] = vd;
    return vel;
}

fsm_state::fsm_state(queryVariable qV)
{
    f_Object = qV.nearObjects[0] ;
    std::cout<<"f_Object: ["<<
               f_Object.pose.position.x<<", "<<
               f_Object.pose.position.y<<"]"<<std::endl;
    std::cout<<"f_Object: ["<<
               f_Object.velocity.linear.x<<", "<<
               f_Object.velocity.linear.y<<"]"<<std::endl;
    fl_Object = qV.nearObjects[1];
    fr_Object = qV.nearObjects[2];
    bl_Object = qV.nearObjects[3];
    br_Object = qV.nearObjects[4];
    ego_lane_id = qV.ego_lane_id;
    egoCar = qV.egoCar;
    objects = qV.objects;
    gap_leftLane = (fl_Object.velocity.linear.x - bl_Object.velocity.linear.x) * TIME_change + (fl_Object.pose.position.x - bl_Object.pose.position.x);
    gap_llateralDis = min(fl_Object.pose.position.y, bl_Object.pose.position.y) - egoCar.pose.position.y;
    gap_rightLane = (fr_Object.velocity.linear.x - br_Object.velocity.linear.x) * TIME_change + (fr_Object.pose.position.x - br_Object.pose.position.x);
    gap_rlateralDis = abs(max(fr_Object.pose.position.y, br_Object.pose.position.y) - egoCar.pose.position.y);

    float d_front = f_Object.pose.position.x - egoCar.pose.position.x;
    float d_left = sqrt(pow(fl_Object.pose.position.x - egoCar.pose.position.x, 2)
                  + pow(fl_Object.pose.position.y - egoCar.pose.position.y, 2));
    float d_right = sqrt(pow(fr_Object.pose.position.x - egoCar.pose.position.x, 2)
                   + pow(fr_Object.pose.position.y - egoCar.pose.position.y, 2));

    float t_front = (d_front) / egoCar.twist.linear.x;
    float t_left = d_left / sqrt(pow(fl_Object.velocity.linear.x - egoCar.twist.linear.x, 2)
                   + pow(fl_Object.velocity.linear.y - egoCar.twist.linear.y, 2));
    float t_right = d_right / sqrt(pow(fr_Object.velocity.linear.x - egoCar.twist.linear.x, 2)
                    + pow(fr_Object.velocity.linear.y - egoCar.twist.linear.y, 2));

    collision= collision_all(qV.objects);

}

void fsm_state::action(){
  //TODO
}

bool fsm_state::collision_check(vaafo_msgs::DetectedObject obstacle){
    // check whether a collision appears in front of EgoCar
    //std::cout<<"entering collision_check part........................"<<std::endl;
    // coordinate transformation
    tf::Quaternion quat;
    tf::quaternionMsgToTF(egoCar.pose.orientation, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    //std::cout<<"yaw: "<<yaw/M_PI*180.0<<std::endl;
    //yaw = yaw - M_PI;
    //double alpha = atan(egoCar.pose.position.y/egoCar.pose.position.x);
    float entity_position_x = obstacle.pose.position.x*(cos(yaw))
            +obstacle.pose.position.y*(sin(yaw))
            -cos(yaw)*egoCar.pose.position.x-sin(yaw)*egoCar.pose.position.y;
    float entity_position_y = -obstacle.pose.position.x*(sin(yaw))
            +obstacle.pose.position.y*(cos(yaw))
            +sin(yaw)*egoCar.pose.position.x-cos(yaw)*egoCar.pose.position.y;

    entity_position_x = -entity_position_x;
    float vox = sqrt(pow(obstacle.velocity.linear.x, 2)+pow(obstacle.velocity.linear.y, 2));
    float vex = sqrt(pow(egoCar.twist.linear.x, 2)+pow(egoCar.twist.linear.y, 2));

    // define the check area
    bool Collision;
    double entity_velocity_x = vox-vex;
    double entity_velocity_y = 0;
    double collision_boundary_x = pow(entity_velocity_x,2)/acc_brake_max+
            egoCar.twist.linear.x*t_refresh + s_emb_min;
    double collision_boundary_y = 1.5;

    /*std::cout<<"vox: "<<vox<<std::endl;
    std::cout<<"vex: "<<vex<<std::endl;
    std::cout<<"orientation: ["<<
               roll<<", "<<
               pitch<<", "<<
               yaw<<", "<<"]"<<std::endl;
    std::cout<<"egoCar: ["<<
               egoCar.pose.position.x<<", "<<
               egoCar.pose.position.y<<"]"<<std::endl;
    std::cout<<"obstacle: ["<<
               obstacle.pose.position.x<<", "<<
               obstacle.pose.position.y<<"]"<<std::endl;

    std::cout<<"collision_boundary_x: "<<collision_boundary_x<<std::endl;
    std::cout<<"entity_velocity_x: "<<entity_velocity_x<<std::endl;
    std::cout<<"entity_velocity_y: "<<entity_velocity_y<<std::endl;
    std::cout<<"entity_position_x: "<<entity_position_x<<std::endl;
    std::cout<<"entity_position_y: "<<entity_position_y<<std::endl;*/

    if(entity_velocity_x <0){
        // check collision for slower obstacle
        if ((entity_position_x>0 && entity_position_x<collision_boundary_x)&& (abs(entity_position_y)<collision_boundary_y)){
            Collision = true;
        }else {
            if (entity_velocity_y==0.0){
                Collision=false;
            }else {
                if(entity_position_x>0 && abs(entity_position_y)>1.5){
                    // traffic participant on both sides (right and left)
                    double t_cut_in = abs(entity_position_y)/entity_velocity_y;
                    // predict the obstacle position after cut in
                    double prediction_pose_x = entity_position_x + t_cut_in*entity_velocity_x;

                    if(prediction_pose_x>0.0 && prediction_pose_x< collision_boundary_x){
                        Collision = true;
                    }else {
                        Collision = false;
                    }
                }else {
                  Collision=false;
                }
            }
        }
    }else {
        Collision=false;
    }
    return Collision;
}

bool fsm_state::collision_all(std::vector<vaafo_msgs::DetectedObject> all_objects){

    int num = int(all_objects.size());
    std::cout<<"number of the objects: "<<num<<std::endl;
    bool collision;
    for(int i=0; i<num; i++){
        collision = collision_check(all_objects.at(i));
        if(collision==true)
            return true;
    }
    return false;
}





// ===============================
// ========= State Buffer ========
// ===============================

void Buffer::action(){
  // accept command from the previous FSM || return
}

fsm_state* Buffer::update_state(queryVariable qV)
{
  cout << "Buffer is updating......" << endl;
  fsm_state* next_state = NULL; 
  // next state option: s1, s17, s18
  if (s1())
  { // next state: Keep Lane
    cout << "next state: KL" << endl;
    next_state = new KL(qV);
  }
  /*else if (s17())
  { // next stste: Prep change Left;
    cout << "next state: PCL" << endl;
    next_state = new PCL(qV);
  }*/
  else if (s18())
  { // next State: Prep change right
    cout << "next state: PCR" << endl;
    next_state = new PCR(qV);
  }
  else if (s19())
  { // next state: Stop
    cout << "next state: Stop" << endl;
    next_state = new Stop(qV);
  }
  else {
	//next state: this state
    cout << "next state: this state" << endl;
    next_state = new Buffer(qV);
  }
  return next_state;
}

bool Buffer::s1(){

  //bool condition = collision== false && turnLeft_Motiv == false && turnRight_Motiv == false;
  bool condition = collision== false ;
    return condition;
}

bool Buffer::s17()
{
  bool condition = collision == false && turnLeft_Motiv == true;
  return condition;
}

bool Buffer::s18()
{
  bool condition = collision == false && turnRight_Motiv == true;
  return condition;
}

bool Buffer::s19()
{
  bool condition = collision == true;
  return condition;
}



// ===============================
// =========== State KL ==========
// ===============================

void KL::action()
{
  // 是否返回信号
  void keepdrive();
  void acceleration();
  void deccleration();
}

bool KL::cost_function()
{

    // need to be read from the path message
    // already in the fast path, don't need to overtake
    int changed_id;
    if (ego_lane_id==-2){
        changed_id =-1;
    }
    else{
        changed_id = 1;
    }

    int cost1;
    float cost2;
    float cost3;
    float w1 = 1.0;
    float w2 = 1.0;
    float w3 = 1.0;
    float path_length = stopline_coord-stopline_last_coord;// the whole length of this straight path
    // cost1 function for lane change
    if (soll_lane_id == changed_id){
        cost1 = 0;
    }else {
        cost1 = 1;
    };
    // cost2 function for speed
    cost2 = sqrt(pow(f_Object.velocity.linear.x, 2)+pow(f_Object.velocity.linear.y, 2))/
            sqrt(pow(egoCar.twist.linear.x,2) +pow(egoCar.twist.linear.y,2));
    double distance = sqrt(pow(f_Object.pose.position.x-egoCar.pose.position.x, 2)+
                           pow(f_Object.pose.position.y-egoCar.pose.position.y, 2));
    // cost3 function for distance
    // stopline here is in the geodetic coordination
    // ratio or relative length
    cost3 = (egoCar.pose.position.x - stopline_last_coord)/path_length;

    //float cost = w1*cost1+ w2*cost2 + w3*cost3;
    float cost =w2*cost2;
    std::cout<<"poseX: "<<f_Object.pose.position.x<<std::endl;
    std::cout<<"[Vo, Ve]: ["<<sqrt(pow(f_Object.velocity.linear.x, 2)+pow(f_Object.velocity.linear.y, 2))
            <<", "<<sqrt(pow(egoCar.twist.linear.x,2) +pow(egoCar.twist.linear.y,2))<<"]"<<std::endl;
    std::cout<<"distance: "<<distance<<std::endl;
    bool judge = (cost <= criterium) && (distance < 20);
    return judge;
}

// TODO：stoplines[next], costfunction
fsm_state* KL::update_state(queryVariable qV)
{
  cout << "State KeepLane is updating......" << endl;
  fsm_state* next_state = NULL;
  // transition condition option: s2, s3, s4, s15
  if (s2())
  {
      startTime_f = -1;
    // transition condition: s2, next state: Buffer
    next_state = new Buffer(qV);
    cout << "next state: Buffer" << endl;
  }
  else if (s3())
  {  // transition condition: s3, next state: Prep change left
    startTime_w = ros::Time::now().toNSec();
    next_state = new PCL(qV);
    cout << "next state: Prep change left" << endl;
    //ros::shutdown();
  }
  else if (s4())
  {
    startTime_w = ros::Time::now().toNSec();
    // transition condition: s4, next state: Prep change right
    next_state = new PCR(qV);
    cout << "next state: Prep change right" << endl;
  }
  else if (s15()){
    // transition condition: s15, next state: Stop
      std::cout<<"######################################STOP##########################################"
                 "#######################################################################################"
                 "#########################################################################################"<<std::endl;
      startTime_f = -1;
      next_state = new Stop(qV);
      cout << "next state: Stop" << endl;
      //ros::shutdown();
  }
  else
  {
    next_state = new KL(qV);
    cout << "next state: KL" << endl;
  }
  return next_state;
}

bool KL::s2()
{
  bool condition = ((stopline_coord - egoCar.pose.position.x) < STOPLINE_near) && collision == false;
  return condition;
}

bool KL::s3()
{
  // add cost function here
  /*bool condition = collision == false && ((stopline_coord - egoCar.pose.position.x) > STOPLINE_far) &&
                   (t_LastFaliure > TIME_interval || t_LastFaliure < 0) &&(ego_lane_id==-2||ego_lane_id==2)&&
                   ((this->turnLeft_Motiv == true) || cost_function() == true);*/
  bool cost = cost_function();
    bool condition = collision == false && ((stopline_coord - egoCar.pose.position.x) > STOPLINE_far) &&
                       (t_LastFaliure > TIME_interval || t_LastFaliure < 0) &&
                       ((turnLeft_Motiv == true) || cost == true);
  return condition;
}

bool KL::s4()
{
  bool condition = collision == false && ((stopline_coord - egoCar.pose.position.x) > STOPLINE_far) &&
                   (t_LastFaliure > TIME_interval || t_LastFaliure < 0) && turnRight_Motiv == true;
  return condition;
}

bool KL::s15()
{
  bool condition = (collision == true);
  return condition;
}



// ===============================
// =========== State PCL =========
// ===============================

void PCL::action()
{
  // Activating the signal light and computing the possibility of turning right/left
  void light_on();
}



fsm_state* PCL::update_state(queryVariable qV)
{
    cout << "State Prep Change Left is updating......" << endl;
    if(startTime_f>0){
        t_LastFaliure = ros::Time::now().toNSec() - startTime_f;
    }
  t_wait = ros::Time::now().toNSec() - startTime_w;
    // transition condition option: s5, s7, s13

  fsm_state* next_state = NULL;
  if (s5())
  {
    // wait time out of limitation, set to -1
    t_wait = -1;
    std::cout<<"t_wait out of s5 :"<<t_wait<<std::endl;
    startTime_w = -1;
    // attempt failed, start t_LastFailure count
    startTime_f = ros::Time::now().toNSec();
    // transition condition: s5, next state: Keep Lane
    next_state = new KL(qV);
    cout << "next state: KL" << endl;
    ros::shutdown();
  }
  else if (s7())
  {  // transition condition: s7, next state: Lane change left
      this->t_wait = -1;
      startTime_w = -1;
      startTime_f = -1;
      t_LastFaliure = -1;
      next_state = new LCL(qV);
      cout << "next state: LCL" << endl;
      ros::shutdown();
  }
  else if (s13())
  {  // transition condition: s13, next state: Stop
      t_wait = -1;
      startTime_w = -1;
      startTime_f = -1;
      t_LastFaliure = -1;
      next_state = new Stop(qV);
      cout << "next state: Stop" << endl;
  }
  else{
      next_state = new PCL(qV);
      cout << "next state: PCL" << endl;
  }
  return next_state;
}

bool PCL::s5()
{
  bool condition = t_wait > TIME_wait && collision == false;
  std::cout<<"t_wait in s5: "<<this->t_wait<<", collision: "<<collision<<std::endl;
  return condition;
}

bool PCL::s7()
{
  //float gap_leftLane = (fl_speed - bl_speed) * TIME_change + (fl_position - bl_position);
    double relDis_fl = sqrt(pow(fl_Object.pose.position.x-egoCar.pose.position.x, 2) +
                      pow(fl_Object.pose.position.y-egoCar.pose.position.y, 2));
    double relDis_bl = sqrt(pow(bl_Object.pose.position.x-egoCar.pose.position.x, 2) +
                         pow(bl_Object.pose.position.y-egoCar.pose.position.y, 2));
    double *vel_ego = getVelocity(egoCar.twist, egoCar.pose.orientation);
    double *vel_fl = getVelocity(fl_Object.velocity, fl_Object.pose.orientation);
    double *vel_bl = getVelocity(bl_Object.velocity, bl_Object.pose.orientation);
    double time_f = relDis_fl / (vel_ego[0] - vel_fl[0]);
    double time_b = relDis_bl / (vel_ego[0] - vel_bl[0]);
    bool condition = (time_b < TIME_change) && (time_f < TIME_change);
  //bool condition = (gap_leftLane >= GAP_vertical) && (gap_llateralDis >= GAP_lateral) && collision == false;
  return condition;
}

bool PCL::s13()
{
  bool condition = collision == true;
  return condition;
}

// ===============================
// =========== State PCR =========
// ===============================

void PCR::action()
{
  // Activating the signal light and computing the possibility of turning right/left
    void light_on();

}


fsm_state* PCR::update_state(queryVariable qV)
{
    if(startTime_f>0){
        t_LastFaliure = ros::Time::now().toNSec() - startTime_f;
    }
  t_wait = ros::Time::now().toNSec() - startTime_w;
  // transition condition option: s6, s8, s14
  cout << "State Prep Change Right is updating......" << endl;
  fsm_state* next_state = NULL;
  if (s6())
  {
    t_wait = -1;
    startTime_w = -1;
    // attempt failed, start t_LastFailure count
    startTime_f = ros::Time::now().toNSec();
    // transition condition: s6, next state: Keep Lane
    next_state = new KL(qV);
    cout << "next state: KL" << endl;
  }
  else if (s8())
  {  // transition condition: s8, next state: Lane change Right
      t_wait = -1;
      startTime_w = -1;
      startTime_f = -1;
      t_LastFaliure = -1;
      next_state = new LCR(qV);
      cout << "next state: LCR" << endl;
  }
  else if (s14())
  {  // transition condition: s14, next state: Stop
      t_wait = -1;
      startTime_w = -1;
      startTime_f = -1;
      t_LastFaliure = -1;
      next_state = new Stop(qV);
      cout << "next state: Stop" << endl;
  }
  else{
      next_state = new PCR(qV);
      cout << "next state: PCR" << endl;
  }
  return next_state;
}

bool PCR::s6()
{
  bool condition = t_wait > TIME_wait && collision == false;
  return condition;
}

bool PCR::s8()
{
  //float gap_rightLane = (fr_speed - br_speed) * TIME_change + (fr_position - br_position);
  bool condition = (gap_rightLane >= GAP_vertical) && (gap_rlateralDis >= GAP_lateral) && collision == false;
  return condition;
}

bool PCR::s14()
{
  bool condition = collision == true;
  return condition;
}

// ===============================
// =========== State LCL =========
// ===============================

void LCL::action()
{
  void turnLeft();
  bool turnleft_success();
}

fsm_state* LCL::update_state(queryVariable qV)
{
  // transition condition option: s9, s11
  cout << "State Lane Change Left is updating......" << endl;
  fsm_state* next_state = NULL;
  if (s9())
  {
    // transition condition: s9, next state: Keep Lane
    next_state = new KL(qV);
    cout << "next state: KL" << endl;
  }
  else if (s11())
  {
    next_state = new Stop(qV);
    cout << "next state: Stop" << endl;
  }
  return next_state;
}

bool LCL::s9()
{
  bool condition = soll_lane_id == ego_lane_id && collision == false;
  return condition;
}

bool LCL::s11()
{
  bool condition = collision == true;
  return condition;
}


// ===============================
// =========== State LCR =========
// ===============================

void LCR::action()
{
  void turnRight();
  bool turnright_success();
}

fsm_state* LCR::update_state(queryVariable qV)
{
  action();
  // transition condition option: s10, s12
  cout << "State Lane Change Right is updating......" << endl;
  fsm_state* next_state = NULL;
  if (s10())
  {
    next_state = new KL(qV);
    cout << "next state: KL" << endl;
  }
  else if (s12())
  {
    next_state = new Stop(qV);
    cout << "next state: Stop" << endl;
  }
  return next_state;
}

bool LCR::s10()
{
  bool condition = soll_lane_id == ego_lane_id && collision == false;
  return condition;
}

bool LCR::s12()
{
  bool condition = collision == true;
  return condition;
}

// ===============================
// ========== State Stop =========
// ===============================
void Stop::action()
{
  // Emergency stop and requesting for a new Trajectory
  void emergency();
}

fsm_state* Stop::update_state(queryVariable qV)
{
  action();
  // transition condition option: s16
  cout << "State Lane Change Right is updating......" << endl;
  fsm_state* next_state = NULL;
  if (s16())
  {
    next_state = new KL(qV);
    cout << "next state: KL" << endl;
  }
  else
  {
    next_state = new Stop(qV);
    cout << "next state: Stop" << endl;
  }
  return next_state;
}

bool Stop::s16()
{
  bool condition = collision==false;
  return condition;
}

//int main(int argv, char **argc){
//    return 0;
//}
