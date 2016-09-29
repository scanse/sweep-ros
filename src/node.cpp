
/*The MIT License (MIT)
 *
 * Copyright (c) 2016, Scanse LLC
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "ros/ros.h"
#include "sweep.h"
#include <pcl/point_types.h>
#include "sensor_msgs/PointCloud2.h"
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

SweepDriver *drv = NULL;
status result;

bool checkDeviceInfo(SweepDriver *drv)
{
    status result;
    sweep_response_info_device_t info_d;
    sweep_response_info_version_t info_v;

    if (drv->getDeviceInfo(&info_d))
    {
        ROS_ERROR_STREAM("Error, couldn't retrieve device info");
    }

    if (drv->getVersionInfo(&info_v))
    {
        ROS_ERROR_STREAM("Error, couldn't retrieve device info");
    }

    if (strncmp((const char *) info_v.model, "SWEEP", 5))
    {
        ROS_ERROR_STREAM("Error, unknown Sweep Model");
        return false;
    }

    if (info_d.diagnostic != '0') //Check Device health is good
    {
        ROS_ERROR_STREAM("Error, device diagnostic info incorrect");
        return false;
    }

    return true;
}

void publish_scan(ros::Publisher *pub,
                  sweep_response_scan_packet_t *nodes,
                  size_t node_count, ros::Time start,
                  double scan_time, std::string frame_id)
{
    pcl::PointCloud <pcl::PointXYZ> cloud;
    sensor_msgs::PointCloud2 cloud_msg;

    float angle_f;
    float x;
    float y;

    cloud.height = 1;
    cloud.width = node_count;
    cloud.points.resize(cloud.width * cloud.height);

    for (size_t i = 0; i < node_count; i++)
    {
        float range = (float) nodes[i].distance;
        angle_f = INT_TO_FLOAT(nodes[i].angle);

        x = (range * cos(DEG2RAD(angle_f))) / 100;
        y = (range * sin(DEG2RAD(angle_f))) / 100;

        cloud.points[i].x = x;
        cloud.points[i].y = y;
    }

    //Convert pcl PC to ROS PC2
    pcl::toROSMsg(cloud, cloud_msg);
    cloud_msg.header.frame_id = "laser_frame";

    ROS_DEBUG("Publishing a full scan");
    pub->publish(cloud_msg);
}


int main(int argc, char *argv[])
{
    //Initialize Node and handles
    ros::init(argc, argv, "sweep_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    //Get Serial Parameters
    std::string serial_port;
    nh_private.param<std::string>("serial_port", serial_port, "/dev/ttyUSB0");
    int serial_baudrate;
    nh_private.param<int>("serial_baudrate", serial_baudrate, 115200);

    //Get Scanner Parameters
    int rotation_speed;
    nh_private.param<int>("rotation_speed", rotation_speed, 5);

    //Get frame id Parameters
    std::string frame_id;
    nh_private.param<std::string>("frame_id", frame_id, "laser_frame");

    //Setup Publisher
    ros::Publisher scan_pub = nh.advertise<sensor_msgs::PointCloud2>("pc2", 1000);

    //Create Sweep Driver Object
    drv = new SweepDriver();

    if (!drv)
    {
        fprintf(stderr, "Create Driver fail, exit\n");
        return -2;
    }

    //Connect to Device
    if (drv->connect(serial_port.c_str(), serial_baudrate, 1000))
    {
        fprintf(stderr, "Error, Serial Couldn't Connect\n");
        delete drv;
        return -1;
    }

    //Send Rotation Speed
    if (drv->changeMotorSpeed(rotation_speed))
    {
        ROS_ERROR_STREAM("Error, couldn't set rotation speed");
    }
    else
    {
        ROS_INFO("expected rotation frequency: %d (Hz)", rotation_speed);
    }

    //Verify Device Info
    if (!checkDeviceInfo(drv))
    {
        ROS_ERROR_STREAM("Error, coudln't verify device info");
    }

    //Start Scan
    drv->startScan();

    ros::Time start_scan_time;
    ros::Time end_scan_time;
    double scan_duration;
    while (ros::ok())
    {
        sweep_response_scan_packet_t nodes[MAX_SCAN_PACKETS];
        size_t count = _countof(nodes);

        //Grab Full Scan
        start_scan_time = ros::Time::now();
        result = drv->getCompleteScan(nodes, count, 3000);
        end_scan_time = ros::Time::now();

        scan_duration = (end_scan_time - start_scan_time).toSec();

        if (result == S_OK)
        {
            publish_scan(&scan_pub, nodes, count, start_scan_time, scan_duration, frame_id);
        }
        else if (result == S_FAIL)
        {
            ROS_ERROR_STREAM("Error, failed scan");
            // The data is invalid, publish anyways
            publish_scan(&scan_pub, nodes, count, start_scan_time, scan_duration, frame_id);
        }
        ros::spinOnce();
    }
    // destroy driver
    delete drv;
    return 0;
}