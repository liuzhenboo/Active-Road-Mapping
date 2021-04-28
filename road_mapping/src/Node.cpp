
#include <System.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "mav_find_road");
	ros::NodeHandle nh("~");
	System mysystem;
	mysystem.Init_parameter(nh);
	mysystem.run();
	return 0;
}