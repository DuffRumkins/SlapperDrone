// Adapated from this video https://www.youtube.com/watch?v=lR3cK9ZoAF8&ab_channel=MahmoudAbdelgalil
// This allows one node to subscribe to a topic and then publish to two separate new topics

#ifndef POSITION_PUBLISHER_SUBSCRIBER_H
#define POSITION_PUBLISHER_SUBSCRIBER_H

#include <ros/ros.h>
#include <string>

template<typename PublishT, typename SubscribeT> //Define template along with types PublishT and SubscribeT
                                                 // These are generic datatypes so variables of all types can be passed to the publisher and
                                                 // subscriber based on when this is called in the main function
class PositionPublisherSubscriber   //Define class PositionPublisherSubscriber
{
public:
    PositionPublisherSubscriber() {} //Declaring structure
    PositionPublisherSubscriber(std::string publishTopicName,std::string subscribeTopicName,
                        int queueSize) //Takes strings and an int as inputs
    {
        posxPublisher = nH.advertise<PublishT>(publishTopicName + "/posx", queueSize); // Position x axis publisher object
        posyPublisher = nH.advertise<PublishT>(publishTopicName + "/posy", queueSize); // Position y axis publisher object
        poszPublisher = nH.advertise<PublishT>(publishTopicName + "/posz", queueSize); // Position z axis publisher object
        orixPublisher = nH.advertise<PublishT>(publishTopicName + "/orix", queueSize); // Orientation x quaternion publisher object
        oriyPublisher = nH.advertise<PublishT>(publishTopicName + "/oriy", queueSize); // Orientation y quaternion publisher object
        orizPublisher = nH.advertise<PublishT>(publishTopicName + "/oriz", queueSize); // Orientation z quaternion publisher object
        oriwPublisher = nH.advertise<PublishT>(publishTopicName + "/oriw", queueSize); // Orientation w quaternion publisher object
        subscriberObject = nH.subscribe<SubscribeT>(subscribeTopicName,queueSize,&PositionPublisherSubscriber::subscriberCallback,this);
        //A subscriber object is also created that subscribes that expects data of generic type SubscribeT. It subscribes to the topic 
        //subscribeTopicName and has the same queue size, queueSize. The &PositionPublisherSubscriber::subscriberCallback,this is the syntax used to
        //reference the callback function and is used so that function can be performed once the subscriber has received data
    }
    void subscriberCallback(const typename SubscribeT::ConstPtr& receivedMsg); //Here the callback function used above is declared and it
                                                                               //takes a constant pointer to aconstant variable of some type
                                                                               //Since it is only declared here the definition will have to 
                                                                               //provided when an object of this class is created in the main
                                                                               //function

protected: //Here the attributes that are used by this class are declared
    //First the subscriber and publisher objects are declared
    ros::Subscriber subscriberObject;

    ros::Publisher posxPublisher;
    ros::Publisher posyPublisher;
    ros::Publisher poszPublisher;
    ros::Publisher orixPublisher;
    ros::Publisher oriyPublisher;
    ros::Publisher orizPublisher;
    ros::Publisher oriwPublisher;

    //Then the node handle is declared
    /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fulpos_y initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
    ros::NodeHandle nH;        
};

#endif