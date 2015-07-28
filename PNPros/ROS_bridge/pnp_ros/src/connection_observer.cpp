#include <pnp_ros/connection_observer.h>

#include <cstdio>
#include <iostream>
#include <sstream>

#include <cstring>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>



ConnectionObserver::ConnectionObserver(const std::string& name, bool use_java_connection) :
    newsockfd_(-1),
    plan_name_(name)
{

    if(use_java_connection)
	{
	    int sockfd = startServer();
	    if(sockfd < 0)
		return;

	    std::cout << "ObserverServer: waiting for connections ... " << std::endl;
	    
	    struct sockaddr_in cli_addr;
	    socklen_t clilen = sizeof(cli_addr);
	    newsockfd_ = accept(sockfd, (struct sockaddr *) &cli_addr, &clilen);

	    if (newsockfd_ < 0)
	    {
		std::cerr << "ObserverServer: ERROR on accept" << std::endl;
	    }
	    
	    std::cout << "ObserverServer: client connected ... " << std::endl;
	    
	}
}

ConnectionObserver::~ConnectionObserver()
{
    if(newsockfd_ >= 0)
    {
        close(newsockfd_);
    }
}

int ConnectionObserver::startServer()
{
	int portno = 47996;
	
	struct sockaddr_in serv_addr;
	
	int sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0)
    {
        std::cerr << "ERROR opening socket" << std::endl;
		return -1;
	}
	
	bzero((char *) &serv_addr, sizeof(serv_addr));
	
	serv_addr.sin_family = AF_INET;
	serv_addr.sin_addr.s_addr = INADDR_ANY;
	serv_addr.sin_port = htons(portno);
	
	
	if (bind(sockfd, (struct sockaddr *) &serv_addr,
        sizeof(serv_addr)) < 0)
    {
        std::cerr << "ERROR on binding" << std::endl;
        close(sockfd);
        return -1;
	}
	
	// std::cout << "ObserverServer: bind OK" << std::endl;
	
	listen(sockfd,5);  // std::cout << "ObserverServer: listen OK" << std::endl;
	return sockfd;
}


void ConnectionObserver::markingChanged(const std::map< std::string, int, std::less< std::string >, std::allocator< std::pair< const std::string, int > > >& new_marking)
{

    if(newsockfd_ < 0)
		return; //no connection, let it go.
	
    std::ostringstream stream;
	
    stream << plan_name_ << "[";
	
    std::map< std::string, int>::const_iterator it = new_marking.begin();
	
    for(bool first = true ;it != new_marking.end(); ++it, first = false)
    {
		stream << ((first)? "" : ",") <<  it->second;
	}
	stream << "]" << std::endl;

    std::string message = stream.str();

    int n = 0;
    do
    {
        n = write(newsockfd_,message.c_str(),message.length());
		message = message.substr(n);
	} while(n == message.length());

}

//int ConnectionObserver::sockfd(ConnectionObserver::startServer());
