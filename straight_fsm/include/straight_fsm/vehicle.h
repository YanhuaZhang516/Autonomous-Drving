#ifndef VEHICLE_H
#define VEHICLE_H
#include <ros/ros.h>
#include <time.h>
#include <iostream>
#include <string>
#include <vector>
#include <boost/variant.hpp>
#include <math.h>
#include <straight_fsm/query_variable.h>
#include <tf/tf.h>


using namespace std;

class fsm_state
{
public:
  float const ttc = 9999;      // time to collision
  float const lc = 9999;       // the distance to collision
  float const STOPLINE_far = 0;  // changing lane actions are not allowed when distance between ego car and stopline is smaller than this
  float const STOPLINE_near = 2;  // GoStraight can not deal with the situation when the distance between ego car and stopline is smaller than this
  float const TIME_interval = 200;  // unit:nsec, the time that must wait because of the previous failure
  float const TIME_wait = 300;  // unit:, nsec, the maximum waiting time, once waiting longer than TIME_wait, report failure
  float const TIME_change = 1;     // expected time of changing lanes
  float const GAP_vertical = 5;  // the minimum vertical distance between the left/right frontal car and the left/right back car in target lane
  float const GAP_lateral = 2;  // the minimum lateral distance between ego car and the calculated gap in the other lane
  float const criterium = 0.6; // the criterium for cost function

  static float t_LastFaliure;  // the time interval from last failure to now, set as 0 when there has been no failure
  static float t_wait;
  static bool turnLeft_Motiv;// not defined
  static bool turnRight_Motiv;// not defined
  static clock_t startTime_w;
  static clock_t startTime_f;
 // bool collision;


  double acc_brake_max=10.0; // the max acceleration to brake
  double t_refresh=0.1; // the time to refresh
  double const s_emb_min=2.0; // the mindest distance between the egocar and the front car
  float stopline_coord=9999; // not defined
  float stopline_last_coord;
  int soll_lane_id;  // not defined
  int ego_lane_id;
  //Ego_coord ego_coord;
  vaafo_msgs::CarInfo egoCar;
  vaafo_msgs::DetectedObject f_Object;
  vaafo_msgs::DetectedObject fl_Object;
  vaafo_msgs::DetectedObject fr_Object;
  vaafo_msgs::DetectedObject bl_Object;
  vaafo_msgs::DetectedObject br_Object;

  std::vector<vaafo_msgs::DetectedObject> objects;
  float gap_leftLane;
  float gap_llateralDis;
  float gap_rightLane;
  float gap_rlateralDis;


  fsm_state(queryVariable qV);
  ~fsm_state();
  virtual void action();
  bool collision_check(vaafo_msgs::DetectedObject obstacle);
  bool collision_all(std::vector<vaafo_msgs::DetectedObject> all_objects);
  bool collision;
  virtual fsm_state* update_state(queryVariable qV) = 0;
};

class Buffer : public fsm_state
{
public:
  using fsm_state::fsm_state;
  void action();
  fsm_state* update_state(queryVariable qV);

private:
  bool s1();
  bool s17();
  bool s18();
  bool s19();
};

class KL : public fsm_state
{
public:
  using fsm_state::fsm_state;
  void action();
  fsm_state* update_state(queryVariable qV);

private:
  //float stopline_coord;  // original stoplines[next], the coordinate of the next (nearest) stopline
  //float ego_coord;       // original ego.position, the coordinate of the ego car
  bool s2();
  bool s3();
  bool s4();
  bool s15();
  bool cost_function();
};

class PCL : public fsm_state
{
public:
  void action();
  void lignt_on_left();
  using fsm_state::fsm_state;
  fsm_state* update_state(queryVariable qV);

private:
  bool s5();
  bool s7();
  bool s13();
};

// class PCR :public PCL,fsm_state
class PCR : public fsm_state
{

public:
    void action();
    void light_on_right();
    using fsm_state::fsm_state;
    fsm_state* update_state(queryVariable qV);

private:
  bool s6();
  bool s8();
  bool s14();
};

class LCL : public fsm_state
{
public:
  void action();
  using fsm_state::fsm_state;
  fsm_state* update_state(queryVariable qV);

private:
  bool s9();
  bool s11();
};

class LCR : public fsm_state
{
public:
  void action();
  using fsm_state::fsm_state;
  fsm_state* update_state(queryVariable qV);

private:
  bool s10();
  bool s12();
};

class Stop : public fsm_state
{
public:
    using fsm_state::fsm_state;
    void action();
    fsm_state* update_state(queryVariable qV);

private:
  bool s16();
};

// ??? should these static variables initialized in a main function?


#endif // VEHICLE_H
