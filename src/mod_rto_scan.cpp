#include <mod_rto_scan/mod_rto_scan.h>

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "mod_rto_scan");

    ros::NodeHandle nh;

    ros::Rate loop_rate(30);

    ModifyRtoScan modify_rto_scan(nh);

    while (ros::ok()) 
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}