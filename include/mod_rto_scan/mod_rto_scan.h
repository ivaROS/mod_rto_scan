#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

class ModifyRtoScan
{
    public:
        ModifyRtoScan(ros::NodeHandle& nh) : nh_(nh)
        {
            ROS_INFO_STREAM_NAMED("Timing", "Creating ModifyRtoScan object");

            // Subscribe to the laser scan topic
            scan_sub_ = nh_.subscribe("/rto/scan", 10, &ModifyRtoScan::scanCallback, this);

            // Publish the modified scan
            mod_scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>("/rto/modified_scan", 10);            
        }

        void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
        {
            ROS_INFO_STREAM_NAMED("Timing", "Received laser scan message");
            float range_max = 5.0;

            // Create a new LaserScan message
            sensor_msgs::LaserScan modified_scan;
            modified_scan.header = scan->header;
            modified_scan.angle_min = scan->angle_min;
            modified_scan.angle_max = scan->angle_max;
            modified_scan.angle_increment = scan->angle_increment;
            modified_scan.range_min = scan->range_min;
            modified_scan.range_max = scan->range_max; // returns 5.0 (the actual max range, not infinity)
            modified_scan.ranges = scan->ranges;
        

            // Modify the ranges
            float eps = 0.01; // epsilon value to avoid infinities
            for (size_t i = 0; i < modified_scan.ranges.size(); ++i)
            {
                ROS_INFO_STREAM_NAMED("Timing", "Modifying range at index " << i << ": " << modified_scan.ranges[i]);

                modified_scan.ranges[i] = std::min(modified_scan.ranges[i], range_max - eps); // remove any infinities in scan

                if (std::isnan(modified_scan.ranges[i]))
                {
                    ROS_INFO_STREAM_NAMED("Timing", "Range is NaN, setting to max range");
                    modified_scan.ranges[i] = range_max - eps;
                } else if (std::isinf(modified_scan.ranges[i]))
                {
                    modified_scan.ranges[i] = range_max;
                } else if (modified_scan.ranges[i] > range_max)
                {
                    modified_scan.ranges[i] = range_max;
                }
            }

            // Publish the modified scan
            mod_scan_pub_.publish(modified_scan);
        }


    private:
        ros::Subscriber scan_sub_;
        ros::Publisher mod_scan_pub_;
        ros::NodeHandle nh_;
};