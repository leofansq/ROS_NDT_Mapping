#include "ndt_mapping.h"

int main(int argc, char **argv)
{

  ros::init(argc, argv, "ndt_mapping");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  ndt_mapping ndt(nh, private_nh);

  ros::spin();

  return 0;
};


