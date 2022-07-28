
#ifndef MESSAGE_DEF_H
#define MESSAGE_DEF_H

// ROS
#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float64MultiArray.h>

//LIO_SAM数据集campus_small_dataset的GPS消息类型
#include <sensor_msgs/NavSatFix.h>
using GpsMsg_lioSam=sensor_msgs::NavSatFix;
using GpsMsg_ulhk=sensor_msgs::NavSatFix;
#endif  // MAPPING_MESSAGE_DEF_H
