#include "ROSConds.h"
#include <PNPros/PNPCondition.h>

using namespace std;

namespace pnpros
{
	ROSConds::ROSConds()
	{
		ros::NodeHandle n;
		
		ROS_INFO("Setting service client for PNPConditionEval.");
		
		client = n.serviceClient<PNPros::PNPCondition>("PNPConditionEval");
		
		PNPros::PNPCondition srv;
		
		srv.request.cond = string("hello");
		
		if (client.call(srv))
		{
			ROS_INFO("Test Condition Checker: %s value: %d.",srv.request.cond.c_str(),srv.response.truth_value);
		}
		else
		{
			ROS_ERROR("Failed to call service conds.");
		}
	}
	
	bool ROSConds::evaluateAtomicExternalCondition(const string& atom)
	{
		PNPros::PNPCondition srv;
		
		srv.request.cond = atom;
		
		if (client.call(srv))
		{
			// ROS_INFO("Cond: %s value: %d ", atom.c_str(), srv.response.truth_value);
		}
		else
		{
			ROS_ERROR("Failed to call service conds.");
			
			return false;
		}
		
		return srv.response.truth_value;
	}
}
