/**
This is the control interface for the DH AG-2E Hand
Author:   Jie Sun
Email:    jie.sun@dh-robotics.com
Date:     2018 April 23

Version 1.0
Copyright @ DH-Robotics Ltd.  
**/

#include <dh_hand_driver/hand_controller.h>
HandController::HandController(ros::NodeHandle n,const std::string &name)
: as_(n, name, boost::bind(&HandController::actuateHandCB, this, _1), false)
{
  n.param<std::string>("serial_port", hand_port_name_, "/dev/DH_hand"); 
  n.param<std::string>("Hand_Model",  Hand_Model_    , "AG-2E"); 
  n.param<double>("WaitDataTime",  WaitDataTime_    , 0.5); 
  ROS_INFO("Hand_model : %s",Hand_Model_.c_str());
  ROS_INFO("serial_port: %s",hand_port_name_.c_str());

   if(!build_conn())
   {
     return;
   }
  initHand();

  ros::ServiceServer service = n.advertiseService("hand_joint_state", &HandController::jointValueCB, this);
  as_.start();
  ros::spin();

}

HandController::~HandController()
{
  closeDevice();
}

bool HandController::jointValueCB(dh_hand_driver::hand_state::Request  &req,
         dh_hand_driver::hand_state::Response &res)
{
  W_mutex.lock();
  readtempdata.DataStream_clear();
  switch(req.get_target)
  {
    case 0: ROS_INFO("request: getMotorForce");     Hand.getMotorForce();     break;
    case 1: ROS_INFO("request: getMotor1Position"); Hand.getMotorPosition(1); break;
    case 2: 
      ROS_INFO("request: getMotor2Position"); 
      if(Hand_Model_=="AG-3E")
      {
        Hand.getMotorPosition(2);
        break;
      }
    default: ROS_ERROR("invalid read command");  W_mutex.unlock();return false; break;
  }
    Writedata(Hand.getStream());
  W_mutex.unlock();

  R_mutex.lock();
    Readdata();
  R_mutex.unlock();
  res.return_data = readtempdata.data[0];
  return true;
}



void HandController::actuateHandCB(const dh_hand_driver::ActuateHandGoalConstPtr &goal)
{
    ROS_INFO("Start to move the DH %s Hand" , Hand_Model_.c_str() );

    dh_hand_driver::ActuateHandFeedback feedback;
    dh_hand_driver::ActuateHandResult result;
    bool succeeded = false;
    bool bFeedback = false;

    // move Motor
    if(Hand_Model_=="AG-2E"&&goal->MotorID==2)
    {
      ROS_ERROR("invalid AG-2E command");
      as_.setAborted(result);
      return;
    }

    if(goal->force >= 0 && goal->position >=0 && goal->position <= 100)
    { 
      setGrippingForce(goal->force);
      succeeded = moveHand(goal->MotorID,goal->position,bFeedback);
      feedback.position_reached = bFeedback;
      as_.publishFeedback(feedback);
      ros::Duration(WaitDataTime_).sleep();
      result.opration_done = succeeded;
    }
    else
    {
     ROS_ERROR("received invalid action command");
    }
    if(succeeded)
      as_.setSucceeded(result);
    else
      as_.setAborted(result);

}

/**
 * To build the communication with the servo motors
 *
* */
bool HandController::build_conn()
{

  bool hand_connected = false;

  try
    {
        hand_ser_.setPort(hand_port_name_);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        hand_ser_.setTimeout(to);
        hand_ser_.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port of the hand");
        return hand_connected;
    }

    if(hand_ser_.isOpen())
    {
        ROS_INFO_STREAM("Serial Port for hand initialized");
        hand_connected = true;
    }
    else
    {
        return hand_connected;
    }
  

  return hand_connected;
}



void HandController::closeDevice()
{
  hand_ser_.close();
  // stop communication
}

void HandController::initHand()
{
   // send initialize commond 
  W_mutex.lock();
    Hand.setInitialize();
    Writedata(Hand.getStream());
  W_mutex.unlock();

  R_mutex.lock();
     Readdata();
  R_mutex.unlock();

    ros::Duration(5).sleep();

  R_mutex.lock();
     Readdata();
  R_mutex.unlock();
}

bool HandController::moveHand(int MotorID, int target_position,bool &feedback)
{
  W_mutex.lock();
  Hand.setMotorPosition(MotorID,target_position);
    Writedata(Hand.getStream());
  W_mutex.unlock();

  R_mutex.lock();
    Readdata();
  R_mutex.unlock();

  ros::Duration(WaitDataTime_).sleep();

  W_mutex.lock();
    Hand.getFeedback(MotorID);
    Writedata(Hand.getStream());
  W_mutex.unlock();

  R_mutex.lock();
  if(Readdata())
  {
      R_mutex.unlock();
    if(readtempdata.data[0]==2)
        feedback = true;
    else
        feedback = false;

    return true;
  }
  else
  {
    R_mutex.unlock();
    return false;
  }
}

void HandController::setGrippingForce(int gripping_force)
{
  W_mutex.lock();
    Hand.setMotorForce(gripping_force);
    Writedata(Hand.getStream());
  W_mutex.unlock();

  R_mutex.lock();
    Readdata();
  R_mutex.unlock();

}

bool HandController::Writedata(std::vector<uint8_t> data)
{
    if(hand_ser_.write(data)==data.size())
    {
      return true;
    }
    else
    {
      return false;
    }
}

// can not process Special case
bool HandController::Readdata()
{ 
  static int HadOvertime = 0;
  
  int i = WaitDataTime_ / 0.05;
  while(i--)
  {
    ros::Duration(0.05).sleep();
    if(hand_ser_.available()<14&&i==1)
    {
      ROS_ERROR_STREAM("Read Overtime you can add WaitDataTime in launch file ");
      HadOvertime++;
      return false;
    }
    else if(hand_ser_.available()>=14)
    {
      uint8_t buf[14];
      if(HadOvertime&&hand_ser_.available()>14)
      {
         ROS_INFO("Read last");
         for(;HadOvertime>0;HadOvertime--)
            hand_ser_.read(buf,14);
      }
      hand_ser_.read(buf,14);
      readtempdata.DatafromStream(buf,14);
      ROS_INFO("Read: %X %X %X %X %X %X %X %X %X %X %X %X %X %X",buf[0],buf[1],buf[2],buf[3],buf[4],buf[5],buf[6],buf[7],buf[8],buf[9],buf[10],buf[11],buf[12],buf[13]);
      return true;
    }
  }
  return false;
}
