/******************************************************************************
* @file conrol.cpp
* @brief This file will control the robot and will act as an intermidiary
*        between the motors and the navigation messages.  This will allow
*        switching between autonomous and teloop conrols when both are 
*        implemented.
* @author Matt Anderson <mia2n4@mst.edu>
* @version 0.1.0
******************************************************************************/

/******************************************************************************
* Ros Includes
******************************************************************************/
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/TwistStamped.h>

/******************************************************************************
* Enumerations
******************************************************************************/
enum Mode
{
    STANDBY,
    AUTONOMOUS,
    TELEOP
}; 

/******************************************************************************
* Global variables
******************************************************************************/
geometry_msgs::TwistStamped twist;
Mode m;

/******************************************************************************
* @fn joy_callback(const sensor_msgs::Joy::ConstPtr& joy)
* @brief Gets information from the joystick and adjusts either the mode or the
*        twist message as necessary
* @pre A valid joy message is required
* @post The global twist message is modified if the mode is teleop or the mode
*       is modified
* @param Expects a ros joy message
******************************************************************************/
void joy_callback(const sensor_msgs::Joy::ConstPtr& joy)
{
    //Check to see if B is pressed, if it is switch to standby
    if (joy->buttons[1] == 1)
    {
        m = STANDBY;
    }
    else if (joy->buttons[2] == 1)
    {
        m = AUTONOMOUS;
    }
    else if (joy->buttons[0] == 1)
    {
        m = TELEOP; 
    }
    
    switch(m)
    {
        case STANDBY:
            twist.twist.linear.x  = 0;
            twist.twist.angular.x = 0;
            break;
        
        case AUTONOMOUS:
            twist.twist.linear.x  = 0;
            twist.twist.angular.x = 0;
            break;

        case TELEOP:
            twist.twist.linear.x  = joy->axes[1];
            twist.twist.angular.x = -joy->axes[0];
            break;
    }
}

/******************************************************************************
* @fn main
* @brief This will start the conrol node and publish commands to the motor node
******************************************************************************/
int main(int argc, char **argv)
{
    //Begin initial ros setup
    ros::init(argc, argv, "control");
    ros::NodeHandle n;
    
    //Create Publishers
    ros::Publisher motor_pub;
    ros::Subscriber joy_sub;

    joy_sub   = n.subscribe<sensor_msgs::Joy>("joy", 1, joy_callback);
    motor_pub = n.advertise<geometry_msgs::TwistStamped>("twist", 1);
    
    //Set mode to stanby to start
    m = STANDBY;

    ros::Rate r(10);
    while (ros::ok())
    {
        //Make decision based on what mode the robot is in
        switch(m)
        {
            case STANDBY:    
                twist.twist.linear.x  = 0;
                twist.twist.angular.x = 0;
                break;

            case AUTONOMOUS:
                //coming soon
                twist.twist.linear.x  = 0;
                twist.twist.angular.x = 0;    
                break;

            case TELEOP:
                break;
                 
            default:
                twist.twist.linear.x  = 0;
                twist.twist.angular.x = 0;
        }

        motor_pub.publish(twist);
        ros::spinOnce();
        r.sleep();
    }
}
