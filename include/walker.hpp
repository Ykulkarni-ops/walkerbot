/*
 *  @file    walker.hpp
 *  @author  Yash Kulkarni
 *  @brief   walker class header 
 *  @License BSD 3 license
 * 
 *  Copyright (c) 2021, Yash Kulkarni
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
#ifndef INCLUDE_WALKER_HPP_
#define INCLUDE_WALKER_HPP_

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

/**
 * @brief Class to implement walker algorithm.
 */
class walker {
 public:
    /**
     * @brief New instance of walker class.
     */
    walker();

    /**
     * @brief Destroys the object of walker class.
     */
    ~walker();

    /**
     * @brief Callback function for subscriber function.
     * @param[in] msg The message subscribed from laser scan topic.
     */
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

    /**
     * @brief Function to stop the bot
     */
    void stop();

    /**
     * @brief Function to move the robot.
     */
    void move();

 private:
    ros::NodeHandle nh;

    geometry_msgs::Twist msg;
    ros::Publisher velocity;
    ros::Subscriber laser;

    bool obstacle_ahead;    
    double obstacle_dis;
    

};

#endif /* INCLUDE_WALKER_HPP_ */
