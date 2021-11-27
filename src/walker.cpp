/*
 *  @file    walker.cpp
 *  @author  Yash Kulkarni
 *  @brief   Implementation of walker algorithm 
 *  @License BSD 3 license
 * 
 *  Copyright (c) 2021, Yash Kulkarni
 *  All rights reserved.
 * 
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 * 
 *  1. Redistributions of source code must retain the above copyright notice, this
 *     list of conditions and the following disclaimer.
 * 
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 * 
 *  3. Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * 
 */


#include <iostream>
#include "walker.hpp"

walker::walker() {
    obstacle_dis = 0.5;
    obstacle_ahead = false;
    //  Publish velocity data into the node.
    velocity = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    // Subscribe to the laser scan message.
    laser = nh.subscribe <sensor_msgs::LaserScan> \
            ("scan", 1, &walker::laserCallback, this);

    stop();
}

walker::~walker() {
    stop();
}

void walker::stop() {
    msg.linear.x = 0.0;
    msg.linear.y = 0.0;
    msg.linear.z = 0.0;
    msg.angular.x = 0.0;
    msg.angular.y = 0.0;
    msg.angular.z = 0.0;
}

void walker::laserCallback(const sensor_msgs::LaserScan::ConstPtr& data) {
    obstacle_ahead = false;
    // locating sensors ranges 0, 30, 330 degress to avoid continuous detection
    auto sensor1 = data->ranges[0];
    auto sensor2 = data->ranges[30];
    auto sensor3 = data->ranges[330];

    if (sensor1 <= obstacle_dis || sensor2 <= obstacle_dis
        || sensor1 <= obstacle_dis) {
        obstacle_ahead = true;
    }
}

void walker::move() {
    ros::Rate rate(10);

    while (ros::ok()) {
        //  Obstacle ahead will rotate the bot
        if (obstacle_ahead == true) {
            stop();
            ROS_INFO("Obstacle ahead");
            msg.linear.x = 0.0;
            msg.angular.z = 0.8;
        // Else move forward
        } else {
            ROS_INFO("Going forward");
            msg.linear.x = 0.15;
            msg.angular.z = 0.0;
        }
        //  Publish the velocity to the robot.
        velocity.publish(msg);
        ros::spinOnce();
        rate.sleep();
    }
}
