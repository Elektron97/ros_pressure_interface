#include <iostream>
#include <stdio.h>
#include <Eigen/Core>  
#include <ros/ros.h>  
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float32MultiArray.h>

int main(int argc, char** argv){




    ros::init(argc, argv, "Controller"); 
    ros::NodeHandle n;

    ros::Publisher act_pub  = n.advertise<std_msgs::Float32MultiArray>("i_support/pressures", 1000);   // Pressure - Actuation vector pubisher



    Eigen::VectorXd p = Eigen::VectorXd::Zero(9);

    std::cout << "Inserisci i valori del vettore p (9 elementi separati da spazi): ";
    for (int i = 0; i < 9; ++i) {
        std::cin >> p(i);
    }
    
    // Create actuation messages Pressure:
    std_msgs::Float32MultiArray act_msg;
    
    act_msg.layout.dim.resize(2);
    act_msg.layout.dim[0].size = 9;
    act_msg.layout.dim[1].size = 1;
    act_msg.data.resize(9);
    
    // Fill actuation msgs
    int index = 0;
    for(index = 0; index < 9; index++){

        act_msg.data[index] = p(index);
    
    }

    act_pub.publish(act_msg);
    return 0;

}
