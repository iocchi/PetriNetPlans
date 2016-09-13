#include <ros/ros.h>
#include <ros/package.h>
#include "std_msgs/String.h"
#include <iostream>
#include <fstream>
#include <pnp_translator/digraph_transl.h>

using namespace std;

class PNP_ROSPlan_Interface{
private:
  ros::NodeHandle nh;
  ros::Subscriber rosplan_sub;
  std_msgs::String plan;
  string file_to_write;

  //write the plan message
  bool write_plan(){
    if(this->plan.data.empty()){
      ROS_ERROR("Plan is empty!");
      return false;
    }
    ofstream f;

    std::string path = ros::package::getPath("rosplan_to_pnp");
    // this->file_to_write = path+"/plans/plan.txt";
    this->file_to_write = path+"/plans/plan.txt";

    f.open(file_to_write.c_str());
    if(!f.good()){
      ROS_ERROR("error while opening the file: %s",this->file_to_write.c_str());
      return false;
    }

    f << this->plan.data.c_str();
    f.close();
    return true;
  }

public:
  void plan_callback(std_msgs::String plan){
      ROS_INFO("Plan received: \n %s",plan.data.c_str());
      this->plan = plan;

      //write the plan message at plans/plan.txt
      if(!write_plan()){
        ROS_ERROR("Problems while writing plan");
        return;
      }
      ROS_INFO("Plan written at: %s",this->file_to_write.c_str());
      string node_path = ros::package::getPath("rosplan_to_pnp");
      string plan_out = node_path+"/plans/plan.txt";
      string pnml_out = node_path+"/plans/";
      //read plan.txt and generate ConditionalPlan
      DigraphTransl tsr(plan_out,pnml_out);
      tsr.read_file();

      //generate the pnml
      // vector<ConditionalPlan> v = tsr.get_plan();
      // tsr.write_pnml(v);
      ROS_INFO("PNML generated");
  }

  PNP_ROSPlan_Interface(){
    rosplan_sub = nh.subscribe("/kcl_rosplan/plan_graph", 10, &PNP_ROSPlan_Interface::plan_callback, this);
    ROS_INFO("waiting for plans...");
  }


};

int main(int argc, char** argv){
  ros::init(argc, argv, "pnp_rosplan_interface");

  PNP_ROSPlan_Interface interface;

  ros::spin();

  return 0;
}
