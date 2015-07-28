#ifndef CONNECTION_OBSERVER__H_
#define CONNECTION_OBSERVER__H_

#include <cstdio>
#include <iostream>
#include <sstream>

#include <cstring>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>

#include<ros/ros.h>
#include<pnp/plan_observer.h>



class ConnectionObserver : public PetriNetPlans::PlanObserver
{
public:
    ConnectionObserver(const std::string& name, bool use_java_connection);
    ~ConnectionObserver();

    void markingChanged(const std::map< std::string, int, std::less< std::string >, std::allocator< std::pair< const std::string, int > > >& new_marking);

private:
    std::string plan_name_;
    int newsockfd_;

    //int sockfd(int start);
    int startServer();


};



#endif
