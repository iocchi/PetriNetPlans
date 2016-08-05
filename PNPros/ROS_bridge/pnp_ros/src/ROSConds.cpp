#include <pnp_ros/ROSConds.h>


using namespace std;

namespace pnpros
{
	ROSConds::ROSConds()
	{
		ros::NodeHandle n;
		
		ROS_INFO("Setting service client for PNPConditionEval.");
		
        client = n.serviceClient<pnp_msgs::PNPCondition>("PNPConditionEval");
		
        pnp_msgs::PNPCondition srv;
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

        int r=-1; bool result=false;

        //cout <<  "    evaluateAtomicExternalCondition: " << atom << " begin ... " << endl;

        // This is necessary because multiple calls to the same condition can happen
        if (ConditionCache.find(atom) != ConditionCache.end()) {
            result = ConditionCache[atom];
            //cout <<  "    evaluateAtomicExternalCondition: " << atom << " CACHED result = " << result << " ... end" << endl;
            return result;
        }


        if (atom.find('@') == std::string::npos) {
            // Try to read condition from ROS parameters
            string rospar = "PNPconditionsBuffer/" + atom;
            ros::param::get(rospar,r);
        }

        if (r==-1) {
            pnp_msgs::PNPCondition srv;

            srv.request.cond = atom;

            // LI DEBUG::: It takes time. Use only if needed!
            bool r = client.call(srv);

            if (r)
            {
                // ROS_INFO("Cond: %s value: %d ", atom.c_str(), srv.response.truth_value);
                result = srv.response.truth_value;
            }
            else
            {
                ROS_ERROR("Failed to call service conds.");
                return false;
            }
        }
        else {
            result = (r==1);
        }

        ConditionCache[atom]=result;

        //cout <<  "    evaluateAtomicExternalCondition: " << atom << " result = " << result << " ... end" << endl;

        return result;
	}
}
