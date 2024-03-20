#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose2D.h>
#include <cmath>
#include <iostream>
#include <eigen3/Eigen/Dense>

float step= 0.01;

float roll = 0;
float pitch = 0;
float yaw = 0;

float roll_des = 0;
float pitch_des = 0;
float yaw_des = 0;

float roll_pun = 0;
float pitch_pun = 0;
float yaw_pun = 0;

float roll_des_pun = 0; 
float pitch_des_pun = 0; 
float yaw_des_pun = 0;

float e_roll = 0;
float e_pitch = 0;
float e_yaw = 0;

float e_roll_pun = 0;
float e_pitch_pun = 0;
float e_yaw_pun = 0;

float u_roll = 0;
float u_pitch = 0;
float u_yaw = 0;

float kp_roll = 70;
float kp_pitch =70;
float kp_yaw = 70;

float kd_roll = 20;
float kd_pitch = 20;
float kd_yaw = 20;

float jxx = 0.0418;
float jyy = 0.0478;
float jzz = 0.0599;

float tor_roll = 0;
float tor_pitch = 0;
float tor_yaw = 0;

//Callbacks

void rpCallback(const geometry_msgs::Vector3::ConstPtr& c) //Viene del Nodo Pos
{
    roll_des = c->y;
    pitch_des = c->z;
}

void yawCallback(const geometry_msgs::Vector3::ConstPtr& c)
{
    yaw_des = c->x;
    yaw_des_pun = c->y;
}

void angCallback(const geometry_msgs::Vector3::ConstPtr& c) //Viene del Nodo Pos
{
    roll = c->x;
    pitch = c->y;
    yaw = c->z;

}


void vangCallback(const geometry_msgs::Vector3::ConstPtr& c) //Viene del Nodo Pos
{
    roll_pun = c->x;
    pitch_pun = c->y;
    yaw_pun = c->z;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "control_ang"); // ros::init(argc, argv, "$<NOMBRE DEL ARCHIVO CPP>")
    ros::NodeHandle nh; //ros::NodeHandle $NOMBRE_DE_LA_VARIABLE_QUE_REPRESENTA_AL_NODO_EN_EL_CÓDIGO>
    
    // Declaración de los publishers
    //ros::Publisher <$PUBLISHER_NAME> = nh.advertise<$TIPO_DE_MENSAJE>("<$TÓPICO>", 
    //<$NÚMERO_DE_DATOS_A_GUARDAR_EN_CASO_DE_TERMINAR_EL_NODO>)
    ros::Publisher torq_pub = nh.advertise<geometry_msgs::Vector3>("/torq", 100);
    ros::Subscriber rollpitch_sub = nh.subscribe("/thrust", 100, &rpCallback);
    ros::Subscriber yaw_sub = nh.subscribe("/yaw", 100, &yawCallback);
    ros::Subscriber ang_sub = nh.subscribe("/ang", 100, &angCallback);
    ros::Subscriber vang_sub = nh.subscribe("/vang", 100, &vangCallback);


    ros::Rate loop_rate(100); 


    geometry_msgs::Vector3 torq_var;

    while (ros::ok())
    {
        e_roll = roll_des - roll;
        e_pitch = pitch_des - pitch;
        e_yaw = yaw_des - yaw;

        e_roll_pun = roll_des_pun - roll_pun;
        e_pitch_pun = pitch_des_pun - pitch_pun;
        e_yaw_pun = yaw_des_pun - yaw_pun;

        u_roll = kp_roll * e_roll + kd_roll * e_roll_pun;
        u_pitch = kp_pitch * e_pitch + kd_pitch * e_pitch_pun;
        u_yaw = kp_yaw * e_yaw + kd_yaw * e_yaw_pun;

        tor_roll = jxx*(((jzz-jyy)/jxx)*(pitch_pun*yaw_pun)+u_roll);
        tor_pitch = jyy*(((jxx-jzz)/jyy)*(roll_pun*yaw_pun)+u_pitch);
        tor_yaw = jzz*(((jyy-jxx)/jzz)*(pitch_pun*roll_pun)+u_yaw);

        torq_var.x = tor_roll;
        torq_var.y = tor_pitch;
        torq_var.z = tor_yaw;

        torq_pub.publish(torq_var);
        
        
        
        ros::spinOnce();
        loop_rate.sleep();


    }

    return 0;



}
