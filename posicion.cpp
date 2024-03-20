//librerias
#include "ros/ros.h"
#include <std_msgs/Float32.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
//Including C++ nominal libraries
#include <iostream>
#include <math.h>
#include <vector>
//Including Eigen library
#include <eigen3/Eigen/Dense>
#include "gazebo_msgs/ModelState.h"
#include <std_msgs/Float32.h>
#include <tf2/LinearMath/Quaternion.h>


//Declaración de funciones
void error_function();
void control_function();
void calc_function();

//Definición de las variables locales generales
double pos_x = 0;
double pos_y = 0;
double pos_z = 0;
double pos_x_des = 0;
double pos_y_des = 0;
double pos_z_des = 0;
double vel_x = 0;
double vel_y = 0;
double vel_z = 0;
double vel_x_des = 0;
double vel_y_des = 0;
double vel_z_des = 0;

double error_pos_x = 0;
double error_pos_y = 0;
double error_pos_z = 0;
double error_vel_x = 0;
double error_vel_y = 0;
double error_vel_z = 0;
double x = 0;
double y = 0;
double z = 0;
double Kpx = 0.06;
double Kpy = 0.07;
double Kpz = 0;

double Kdx = 0.6;
double Kdy = 0.6;
double Kdz = 1.6;

double Uvx = 0;
double Uvy = 0;
double Uvz = 0;
double phi = 0;
double theta = 0;
double psi = 0;
double m = 2;
double thrust = 0;
double g = 9.81;
double phi_des=0;
double theta_des=0;
double psi_des=0;
double step = 0.01;
double arg_theta_des =0;
double arg_phi_des = 0;

void posdesCallback(const geometry_msgs::Vector3::ConstPtr& m)
{
    pos_x_des = m->x;  
    pos_y_des = m->y;   
    pos_z_des = m->z;
}

void veldesCallback(const geometry_msgs::Vector3::ConstPtr& p)
{
    vel_x_des = p->x;
    vel_y_des = p->y;
    vel_z_des = p->z;
}

void posCallback(const geometry_msgs::Vector3::ConstPtr& pr)
{
    pos_x = pr->x;
    pos_y = pr->y;
    pos_z = pr->z;
}

void velCallback(const geometry_msgs::Vector3::ConstPtr& pr)
{
    vel_x = pr->x;
    vel_y = pr->y;
    vel_z = pr->z;
}

void angular_positionCallback(const geometry_msgs::Vector3::ConstPtr& pr)
{
    phi = pr->x;
    theta = pr->y;
    psi = pr->z;
}



int main(int argc, char** argv)
{
    // Inicializar ROS y la creación del nodo
    ros::init(argc, argv, "control_lineal"); 
    ros::NodeHandle nh; 
    ros::Rate loop_rate(100);  

    ros::Subscriber pos_des_sub = nh.subscribe("/pos", 100, &posdesCallback);
    ros::Subscriber vel_des_sub = nh.subscribe("/vel", 100, &veldesCallback);
    ros::Subscriber pos_sub = nh.subscribe("/pos_in", 100, &posCallback); //topics
    ros::Subscriber vel_sub = nh.subscribe("/vel_in", 100, &velCallback);
    ros::Subscriber angular_position_sub = nh.subscribe("/ang", 100, &angular_positionCallback);

    //ros::Publisher error_pos_pub = nh.advertise<geometry_msgs::Vector3>("error_pos_x",100);
    //ros::Publisher error_vel_pub = nh.advertise<geometry_msgs::Vector3>("error_vel_x",100);
    ros::Publisher thrust_pub = nh.advertise<geometry_msgs::Vector3>("/thrust",100);
  
    // Definición de las variables que identifican al mensaje de ROS
    //geometry_msgs::Vector3 error_pos_var;
    //geometry_msgs::Vector3 error_vel_var;
    geometry_msgs::Vector3 thrust_var;
        

    while(ros::ok())
    {

    error_function();
    control_function();
    calc_function();

    /*
    error_vel_var.x = error_vel_x;
    error_vel_var.y = error_vel_y;
    error_vel_var.z = error_vel_z;
    */

    thrust_var.x = thrust;
    thrust_var.y = phi_des;
    thrust_var.z = theta_des;

    thrust_pub.publish(thrust_var);
    
    ros::spinOnce();
	loop_rate.sleep();
    }

    return 0;
}

void error_function()
{
    error_pos_x = pos_x_des - pos_x;
    error_pos_y = pos_y_des - pos_y;
    error_pos_z = pos_z_des - pos_z;

    error_vel_x = vel_x_des - vel_x;
    error_vel_y = vel_y_des - vel_y;
    error_vel_z = vel_z_des - vel_z;
}

void control_function()
{
    Uvx = Kpx*error_pos_x + Kdx*error_vel_x;
    Uvy = Kpy*error_pos_y + Kdy*error_vel_y;
    Uvz = Kpz*error_pos_z + Kdz*error_vel_z;
}

void calc_function()
{
    thrust = (m/(cos(phi)*cos(theta)))*(-g+Uvz);
    if (thrust > 0)
    {
        thrust = 0.00001;
    }
    else if (thrust < -30)
    {
        thrust = -30;
    }

    arg_phi_des = ((m/thrust)*((sin(psi_des)*(Uvx))-(cos(psi_des)*(Uvy))+0.0001));
    if (arg_phi_des > 1)
    {
        arg_phi_des = 1;
    }
    else if (arg_phi_des < -1)
    {
       arg_phi_des = -1;
    }
    phi_des = asin (arg_phi_des);

    arg_theta_des =  ((((m/thrust)*Uvx)-sin(psi_des)*sin(phi_des))/((cos(psi_des))*cos(phi_des))+0.00001);
    if (arg_theta_des > 1)
    {
        arg_theta_des = 1;
    }
    else if (arg_theta_des < -1)
    {
       arg_theta_des = -1;
    }
    theta_des = asin(arg_theta_des);

}
