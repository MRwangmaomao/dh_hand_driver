#include <ros/ros.h>
#include <dh_hand_driver/ActuateHandAction.h>
#include <dh_hand_driver/hand_state.h>
#include <actionlib/client/simple_action_client.h>
#include "std_msgs/Int8.h"

typedef actionlib::SimpleActionClient<dh_hand_driver::ActuateHandAction> Client;


 


class DH_HandActionClient {
private:
    // Called once when the goal completes
    void DoneCb(const actionlib::SimpleClientGoalState& state,
            const dh_hand_driver::ActuateHandResultConstPtr& result) {
        ROS_INFO("Finished in state [%s]", state.toString().c_str());
        ROS_INFO("result  : %i", result->opration_done);
    }

    // when target active, call this once
    void ActiveCb() {
        ROS_INFO("Goal just went active");
    }

    // received feedback
    void FeedbackCb(
            const dh_hand_driver::ActuateHandFeedbackConstPtr& feedback) {
        ROS_INFO("Got Feedback: %i", feedback->position_reached);
    }
public:
    DH_HandActionClient(const std::string client_name, bool flag, ros::NodeHandle n) :
            client(client_name, flag), n_(n) {
        client1 = n_.serviceClient<dh_hand_driver::hand_state>("hand_joint_state");

        dh_hand_sub = n_.subscribe<std_msgs::Int8>("/dh_hand", 1,  boost::bind(&DH_HandActionClient::graspOnceCallback, this,_1));
    }
  
    
    //client start
    void Start(int32_t motorID , int32_t setpos,int32_t setforce) {
        ROS_INFO("wait server");
        client.waitForServer();
        //set goal
        dh_hand_driver::ActuateHandGoal goal;
        // AG2E just has one motor (ID:1)
        // AG3E has two motor (ID:1 and 2)
        goal.MotorID    = motorID;
        goal.force      = setforce;
        goal.position   = setpos;

        ROS_INFO("Send goal");
        //sen goal
        client.sendGoal(goal,
                boost::bind(&DH_HandActionClient::DoneCb, this, _1, _2),
                boost::bind(&DH_HandActionClient::ActiveCb, this),
                boost::bind(&DH_HandActionClient::FeedbackCb, this, _1));
        ROS_INFO("wait result");
 
        client.waitForResult(ros::Duration(10.0));

        //process the result
        if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("Send commond succeeded");
        else {
            ROS_INFO("Cancel Goal!");
            client.cancelAllGoals();
        }

        printf("Current State: %s\n", client.getState().toString().c_str());
    }
    
    // grasp once
    void graspOnceCallback(const std_msgs::Int8::ConstPtr& dhhand_msg)
    {
        Start(1,dhhand_msg->data,10);
 
        // use service to get hand state
        
        
        srv.request.get_target = 0;
        if (client1.call(srv))
        {
            ROS_INFO("force: %d",srv.response.return_data);
        }
        else
        {
            ROS_ERROR("Failed to call service");
            return;
        }
        srv.request.get_target = 1;
        if (client1.call(srv))
        {
            ROS_INFO("pos_1: %d",srv.response.return_data);
        }
        else
        {
            ROS_ERROR("Failed to call service");
            return;
        }
    }

private:
    Client client;
    dh_hand_driver::hand_state srv;
    ros::NodeHandle n_;
    ros::ServiceClient client1;
    ros::Subscriber dh_hand_sub; /// 接收电爪消息
};

 
int main(int argc, char** argv) {
    ros::init(argc, argv, "hand_user_client");
     
    ROS_INFO("starting");
    ros::NodeHandle n;
    
    ROS_INFO("starting client");
     
    DH_HandActionClient actionclient("actuate_hand", true, n);
  
    ros::spin(); 
    ros::shutdown();
    return 0;
}
