#include <ros/ros.h> 
#include <geometry_msgs/Vector3.h>
#include <cmath>
#include <iostream>
#include <eigen3/Eigen/Dense>

float m = 2;
float g = 9.81;
float step = 0.01;
float thrust = -(m * g);
float jxx = 0.0411;
float jyy = 0.0478;
float jzz = 0.0599;

Eigen::Vector3f w(0,0,0); 
Eigen::Vector3f w_pun(0,0,0);

Eigen::Vector3f ang(0,0,0);
Eigen::Vector3f ang_pun(0,0,0);

Eigen::Vector3f f(0,0,0);

Eigen::Vector3f ac_loc(0,0,0);
Eigen::Vector3f v_loc(0,0,0);
Eigen::Vector3f v_in(0,0,0);
Eigen::Vector3f pos_lin(-5,0,0);
Eigen::Vector3f torq(0,0,0);

Eigen::Vector3f e3(0,0,1);
Eigen::Matrix3f J;

Eigen::Matrix3f asimetrica(Eigen::Vector3f w){
    Eigen::Matrix3f sk;
    sk << 0, -w(2), w(1), 
        w(2), 0, -w(0),
        -w(1), w(0), 0;
    return sk;
}

Eigen::Matrix3f rotacional(Eigen::Vector3f ang){
    Eigen::Matrix3f r;
    r << cos(ang(2))*cos(ang(1)), cos(ang(2))*sin(ang(0))*sin(ang(1)) - cos(ang(0))*sin(ang(2)), sin(ang(2))*sin(ang(0)) + cos(ang(2))*cos(ang(0))*sin(ang(1)),
        cos(ang(1))*sin(ang(2)), cos(ang(2))*cos(ang(0)) + sin(ang(2))*sin(ang(0))*sin(ang(1)), cos(ang(0))*sin(ang(2))*sin(ang(1)) - cos(ang(2))*sin(ang(0)),
        -sin(ang(1)), cos(ang(1))*sin(ang(0)), cos(ang(0))*cos(ang(1));
    return r;
}

Eigen::Matrix3f rDos(Eigen::Vector3f ang){
    Eigen::Matrix3f r2;
    r2 << 1, sin(ang(0))*tan(ang(1)), cos(ang(0))*tan(ang(1)),
            0, cos(ang(0)), -sin(ang(0)), 
            0, sin(ang(0))/cos(ang(1)), cos(ang(0))/cos(ang(1));
    return r2;
}

        
void posControlCallBack(const geometry_msgs::Vector3::ConstPtr& th){
    thrust = th->x;
}

void angControlCallBack(const geometry_msgs::Vector3::ConstPtr& trq){
    torq(0) = trq->x;
    torq(1) = trq->y;
    torq(2) = trq->z;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dynamics");
    ros::NodeHandle nh;

    ros::Subscriber posControl_sub = nh.subscribe("/thrust", 100, &posControlCallBack);
    ros::Subscriber angControl_sub = nh.subscribe("/torq", 100, &angControlCallBack);

    ros::Publisher pos_pub = nh.advertise<geometry_msgs::Vector3>("/pos_in",100);
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Vector3>("/vel_in",100);
    ros::Publisher ang_pub = nh.advertise<geometry_msgs::Vector3>("/ang",100);
    ros::Publisher velang_pub = nh.advertise<geometry_msgs::Vector3>("/vang",100);


    ros::Rate loop_rate(100);

    geometry_msgs::Vector3 pos_var;
    geometry_msgs::Vector3 vel_var;
    geometry_msgs::Vector3 ang_var;
    geometry_msgs::Vector3 velang_var;

    J << jxx, 0, 0, 
        0, jyy, 0, 
        0, 0, jzz;

    pos_lin << -5, 0, 0;
    w << 0, 0, 0;
    w_pun << 0, 0, 0;
    ang << 0, 0, 0;
    ang_pun << 0, 0, 0;

    v_loc << 0, 0, 0;
    v_in << 0, 0, 0;
    v_in << 0, 0, 0;

    ac_loc << 0, 0, 0;

    pos_var.x = pos_lin(0);
    pos_var.y = pos_lin(1);
    pos_var.z = pos_lin(2);

    vel_var.x = v_in(0);
    vel_var.y = v_in(1);
    vel_var.z = v_in(2);

    ang_var.x = ang(0);
    ang_var.y = ang(1);
    ang_var.z = ang(2);

    velang_var.x = ang_pun(0);
    velang_var.y = ang_pun(1);
    velang_var.z = ang_pun(2);

    pos_pub.publish(pos_var);
    vel_pub.publish(vel_var);
    ang_pub.publish(ang_var);
    velang_pub.publish(velang_var);

    while(ros::ok()){

        //Angular
        w_pun = J.inverse()*(torq - (asimetrica(w) * J * w));

        w(0) = w(0) + step * w_pun(0);
        w(1) = w(1) + step * w_pun(1);
        w(2) = w(2) + step * w_pun(2);

        ang_pun = rDos(ang) * w;

        ang(0) = ang(0) + step * ang_pun(0);
        ang(1) = ang(1) + step * ang_pun(1);
        ang(2) = ang(2) + step * ang_pun(2);

        //Lineal
        f = (thrust * e3) + (rotacional(ang).transpose() * (m * g * e3));
        
        ac_loc = f/m - asimetrica(w) * v_loc;

        v_loc(0) = v_loc(0) + step * ac_loc(0);
        v_loc(1) = v_loc(1) + step * ac_loc(1);
        v_loc(2) = v_loc(2) + step * ac_loc(2);

        v_in = rotacional(ang) * v_loc;

        pos_lin(0) = pos_lin(0) + step * v_in(0);
        pos_lin(1) = pos_lin(1) + step * v_in(1);
        pos_lin(2) = pos_lin(2) + step * v_in(2);
        
        if (pos_lin(0)> 5.5) {
            pos_lin(0)= 5;

        }
        if (pos_lin(0)< -5.5) {
            pos_lin(0)= -5;

        }
        if (pos_lin(1)> 5.5) {
            pos_lin(1)= 5;

        }
        if (pos_lin(1)< -5.5) {
            pos_lin(1)= -5;

        }
        if (pos_lin(2)> 3) {
            pos_lin(2)= 3;

        }
        if (pos_lin(2)< 2) {
            pos_lin(2)= 2;

        }

         
        pos_var.x = pos_lin(0);
        pos_var.y = pos_lin(1);
        pos_var.z = pos_lin(2);

        vel_var.x = v_in(0);
        vel_var.y = v_in(1);
        vel_var.z = v_in(2);

        ang_var.x = ang(0);
        ang_var.y = ang(1);
        ang_var.z = ang(2);

        velang_var.x = ang_pun(0);
        velang_var.y = ang_pun(1);
        velang_var.z = ang_pun(2);

        pos_pub.publish(pos_var);
        vel_pub.publish(vel_var);
        ang_pub.publish(ang_var);
        velang_pub.publish(velang_var);

        ros::spinOnce();
        loop_rate.sleep(); 
        
    }
    return 0;
}
