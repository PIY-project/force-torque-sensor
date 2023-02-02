
#include <ros/ros.h>
#include <numeric>
#include<geometry_msgs/WrenchStamped.h>
#include<rpwc_msgs/ScalarSignalReq.h>

std::vector<double> vecMeas_;
std_msgs::Float64 filteredMeas_;

bool callbackServerScalarSensor(rpwc_msgs::ScalarSignalReq::Request  &req, rpwc_msgs::ScalarSignalReq::Response &res)
{
    res.data = filteredMeas_;
	return true;
}

void callback_wrench_ati (const geometry_msgs::WrenchStampedConstPtr& msg)
{
    if(vecMeas_.size() < 10) vecMeas_.push_back(msg->wrench.force.z);
    else
    {
        vecMeas_.erase(vecMeas_.begin());
        vecMeas_.push_back(msg->wrench.force.z);
        filteredMeas_.data = std::accumulate(vecMeas_.begin(), vecMeas_.end(), 0.0) / vecMeas_.size();
    }
}

int main(int argc, char** argv)
{
	//----------------------------------------------------------
	// Preparations
	//----------------------------------------------------------

	// Initialize the node.
	ros::init(argc, argv, "ati_bridge_node");
	ros::NodeHandle nh;

	ros::ServiceServer serverScalarSensor = nh.advertiseService("scalarSensor", callbackServerScalarSensor);

    ros::Subscriber sub_wrench_ati = nh.subscribe("ft_sensor_hw/ati_FT", 1, &callback_wrench_ati);
	
    ros::spin();
	return 0;
}
