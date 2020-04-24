#include <sstream>

#include <ros/ros.h>


// defined in robotpose.cpp
void exec_gotopose(std::string robotname, float GX, float GY, float GTh, bool *run);
void setNamespace(std::string ns);

int main(int argc, char** argv){

  if (argc<5) {
  	std::cout << "Use: " << argv[0] << " <robotname> <X> <Y> <Theta [DEG]>" << std::endl; 
    exit(-1);
  }

  std::string robotname = std::string(argv[1]);
  double GX = atof(argv[2]), GY = atof(argv[3]), GTh = atof(argv[4]);

  // Init ROS node
  std::ostringstream ss;
  ss << "gotopose_" << robotname << "_" << GX << "_" << GY << "_" << GTh;
  std::string nodename = ss.str();
  ros::init(argc, argv, nodename);

  if (robotname!="" && robotname!="none")  // multi-robot: use robotname prefix
    setNamespace(robotname);

  bool run = true;
  exec_gotopose(robotname, GX, GY, GTh, &run);

  return 0;
}

