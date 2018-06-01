#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 
#include "std_msgs/String.h"
#include <sstream>
#define MESSAGE_FREQ 100

void error(const char *msg) {
    perror(msg);
    exit(0);
}

class Listener {
private:
    char topic_message[256] = { 0 };
public:
    void callback(const std_msgs::String::ConstPtr& msg);
};

void Listener::callback(const std_msgs::String::ConstPtr& msg) {
    memset(topic_message, 0, 256);
    strcpy(topic_message, msg->data.c_str());
    //ROS_INFO("I heard:[%s]", msg->data.c_str());
}

int main(int argc, char *argv[]) {
	ros::init(argc, argv, "client_node");
	ros::NodeHandle nh;

    ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("chatter", 1000);

    ros::Rate loop_rate(MESSAGE_FREQ); // set the rate as defined in the macro MESSAGE_FREQ
	Listener listener;

    int sockfd, portno, n;
    struct sockaddr_in serv_addr;
    struct hostent *server;
    char buffer[256];

    if (argc < 3) {
       fprintf(stderr,"Usage: $ rosrun openvibe_to_ros_tcp client_node <hostname> <port>\n");
       exit(0);
    }
    
    portno = atoi(argv[2]);
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) 
        error("ERROR opening socket");

    server = gethostbyname(argv[1]);
    bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    bcopy((char *)server->h_addr, 
         (char *)&serv_addr.sin_addr.s_addr,
         server->h_length);
    serv_addr.sin_port = htons(portno);
    if (connect(sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0) 
        error("ERROR connecting");

    std_msgs::String message;
    std::stringstream ss;

	while(ros::ok()) {
        ss.str(std::string()); //Clear contents of string stream
        bzero(buffer,256);
        n = read(sockfd,buffer,255);
        if (n < 0) error("ERROR reading from socket");

        ss << static_cast<int>(buffer[0]);
        message.data = ss.str();

        //ROS_INFO("%s", message.data.c_str());
        chatter_pub.publish(message);

	    ros::spinOnce();
	}
	return 0;
}
