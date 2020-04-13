
/*
#include "../include/robotiq/gripper_ur_control.h" // use <robotiq_85_control/gripper_ur_control.h> in your node include, when working in your own package
int main(int argc, char* argv[]){

	ros::init(argc, argv, "gripper_test"); // init ROS nodes
	ros::NodeHandle n; 
	GripperUR g; //create an object of GripperUR class
	g.setCheckpointAddress(15); //not required
	g.init(); //inits gripper
	g.close(); //closes gripper
	g.moveto(50); //moves gripper to a position 0-255
	g.setTimeOut(10); 
	g.moveto(300); //valid invalid
	g.setSpeed(0); //0-255
	g.moveto(3);
	g.setForce(257); //valid invalid 0-255
	g.moveto(100);
	g.open();
	g.setSpeed(300); //valid invalid 0-255
	g.moveto(255);
	g.setSpeed(100);
	g.setForce(0);
	g.moveto(0);
	
	ros::Duration(1.0).sleep();
	
	return 0;
}
*/

#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include <iostream>
#include "../include/robotiq/gripper_ur_control.h" // use <robotiq_85_control/gripper_ur_control.h> in your node include, when working in your own package

class xboxToTwist
{
public:
  xboxToTwist() : spinner_(1)
  {
    g.setCheckpointAddress(15); //not required
    g.init(); //inits gripper
    g.open();
    g.setTimeOut(5); 


    sub_ = n_.subscribe("gripper_state", 1, &xboxToTwist::callback, this);
    spinner_.start();
    ros::waitForShutdown();
  };

private:
  ros::NodeHandle n_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
  ros::AsyncSpinner spinner_;
  GripperUR g; //create an object of GripperUR class



  void callback(const std_msgs::Float64MultiArray::ConstPtr& msg)
  {
	std::cout << msg->data[0] << std::endl;
        ros::Rate loop_rate(0.5);
	if (msg->data[0] == 1){
			g.open(); //closes gripper

	}
	if (msg->data[0] == -1){
			g.close(); //closes gripper

	}
	loop_rate.sleep();

	/*
	g.moveto(50); //moves gripper to a position 0-255
	g.setTimeOut(10); 
	g.moveto(300); //valid invalid
	g.setSpeed(0); //0-255
	g.moveto(3);
	g.setForce(257); //valid invalid 0-255
	g.moveto(100);
	g.open();
	*/	
	
    //loop_rate.sleep();

  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "jogjoint_node");

  xboxToTwist to_twist;

  return 0;
}


