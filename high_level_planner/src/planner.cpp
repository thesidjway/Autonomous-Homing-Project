#include <ros/ros.h>
#include <iostream>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <mutex>
#include "ardrone_autonomy/Navdata.h"

#define PI 3.14159

using namespace std;

float currAngle=0;

std::mutex vel_lock, currAngleLock;

geometry_msgs::Twist vel_msg;

float newTan(float a,float b)
{
	return atan2(a,b)+(atan2(a,b)>0)*2*PI;
}

float theta_1,theta_2,theta_3,theta_1s,theta_2s,theta_3s,beta_32,beta_21,beta_13,beta_13s,beta_32s,beta_21s;


void magCallBack(const geometry_msgs::Vector3Stamped::ConstPtr& msg) 
{
  currAngleLock.lock();
  geometry_msgs::Vector3 magData=msg->vector;
  float x_val=magData.x;
  float y_val=magData.y;
  currAngle=atan2(y_val,x_val)*180.0/PI;
  currAngleLock.unlock();
}

void angleCallback(const geometry_msgs::Twist::ConstPtr& msg) 
{ 


	theta_1 = msg->linear.x;
	theta_2 = msg->linear.y;
	theta_3 = msg->linear.z;

	theta_1s= msg->angular.x;
	theta_2s= msg->angular.y;
	theta_3s= msg->angular.z;

    cout<<"thetas: \t"<<theta_1<<" "<<theta_2<<" "<<theta_3<<endl;
    cout<<"thetaStars: \t"<<theta_1s<<" "<<theta_2s<<" "<<theta_3s<<endl;
    currAngleLock.lock();
    cout<<"currAngle \t"<<currAngle<<endl;
    currAngleLock.unlock();

	beta_21 = theta_2 - theta_1;
	beta_21 = atan2(sin(beta_21),cos(beta_21));

	beta_32 = theta_3 - theta_2;
	beta_32 = atan2(sin(beta_32),cos(beta_32));

	beta_13 = theta_1 - theta_3;
	beta_13 = atan2(sin(beta_13),cos(beta_13));
    
    beta_21s = theta_2s - theta_1s;
    beta_21s = atan2(sin(beta_21s),cos(beta_21s));
    
    beta_32s = theta_3s - theta_2s;
    beta_32s = atan2(sin(beta_32s),cos(beta_32s));
    
    beta_13s = theta_1s - theta_3s;
    beta_13s = atan2(sin(beta_13s),cos(beta_13s));
    

    float OP1[2] = {cos(theta_1), sin(theta_1)};
    float OP2[2] = {cos(theta_2), sin(theta_2)};
    float OP3[2] = {cos(theta_3), sin(theta_3)};
    
    float M[3][3] = {
    	{2*cos(beta_13/2)*(beta_13 - beta_13s),0,2*cos(beta_13/2)*(beta_13 - beta_13s)},
    	{2*cos(beta_21/2)*(beta_21 - beta_21s),2*cos(beta_21/2)*(beta_21 - beta_21s),0},
    	{0,2*cos(beta_32/2)*(beta_32 - beta_32s),2*cos(beta_32/2)*(beta_32 - beta_32s)}
    };
    float N[3][2] = {
    	{cos(theta_1), sin(theta_1)},
    	{cos(theta_2), sin(theta_2)},
   	 	{cos(theta_3), sin(theta_3)}
	};
	float MULT[3][2]={{0,0},{0,0},{0,0}};


	for(int i = 0; i < 3; ++i)
	{
        for(int j = 0; j < 2; ++j)
        {
            for(int k = 0; k < 3; ++k)
            {
                MULT[i][j] += M[i][k] * N[k][j];
            }
        }
	}
    float vel[2]={0,0};

    for(int i=0;i<3;i++)
    {
        currAngleLock.lock();
        vel[0]+=(-MULT[i][0]*sin(currAngle)+MULT[i][1]*cos(currAngle));
        vel[1]+=(MULT[i][0]*cos(currAngle)+MULT[i][1]*sin(currAngle));
        currAngleLock.unlock();

    }
    cout<<"vels: \t\t"<<vel[0]<<" "<<vel[1]<<endl;
    
    vel_lock.lock();
    vel_msg.linear.x=3.0*vel[0];
    vel_msg.linear.y=3.0*vel[1];
    vel_lock.unlock();

    
}

int main(int argc, char **argv)
{   
    ros::init(argc, argv, "homing_node");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("thetaAngles", 30, angleCallback);
    ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("vels", 30);
    ros::Subscriber magSub = n.subscribe <geometry_msgs::Vector3Stamped>("/ardrone/mag", 100, magCallBack); 
    ros::Rate loop_rate(30);
    while(ros::ok())
    {
        vel_lock.lock();
        vel_pub.publish(vel_msg);
        vel_lock.unlock();
        ros::spinOnce();
        loop_rate.sleep();
    }

}

    
 


