#include <ros/ros.h>
#include <straight_fsm/vehicle.h>
#include <straight_fsm/query_variable.h>


int main(int argv, char **argc){

    ros::init(argv, argc, "straightfsm_node");
    ros::Time::init();
    ros::Rate loop_rate(10);

    queryVariable qV;



    fsm_state* state = new Buffer(qV);

    while(ros::ok()){

        qV.proccessData();
        //int a = qV.getFrontLeft();
        //int a = qV.getFrontRight();
        //int a = qV.getBackLeft();
        //int a = qV.getFront();
        //int a = qV.getBackRight();

        state->action();
        fsm_state* nextState = state->update_state(qV);
        state = nextState;


        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
