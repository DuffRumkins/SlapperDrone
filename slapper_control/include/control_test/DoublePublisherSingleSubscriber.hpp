// Adapated from this video https://www.youtube.com/watch?v=lR3cK9ZoAF8&ab_channel=MahmoudAbdelgalil
// This allows one node to subscribe to a topic and then publish to two separate new topics

#ifndef PUBLISHER_SUBSCRIBER_H
#define PUBLISHER_SUBSCRIBER_H

#include <ros/ros.h>
#include <string>

template<typename PublishT1, typename PublishT2, typename SubscribeT> 
                                                 //Define template along with publish types PublishT1 and PublishT2 
                                                 // and subscribe types SubscribeT.
                                                 // These are generic datatypes so variables of all types can be passed to the publisher and
                                                 // subscriber based on when this is called in the main function
class DoublePublisherSingleSubscriber   //Define class DoublePublisherSingleSubscriber
{
public:
    DoublePublisherSingleSubscriber() {} //Declaring structure
    DoublePublisherSingleSubscriber(std::string publishTopicName,std::string secondPublishTopicName,
                        std::string subscribeTopicName, int queueSize) //Takes strings and an int as inputs
    {
        publisherObject = nH.advertise<PublishT1>(publishTopicName, queueSize); //Create publisher that publishes generic type PublishT to
                                                                               //topic publishTopicName with a queue of size queueSize
        secondPublisherObject = nH.advertise<PublishT2>(secondPublishTopicName, queueSize); //Create second publisher object the same as the 
                                                                                           // first except it publishes to secondPublishName
        subscriberObject = nH.subscribe<SubscribeT>(subscribeTopicName,queueSize,&DoublePublisherSingleSubscriber::subscriberCallback,this);
        //A subscriber object is also created that subscribes that expects data of generic type SubscribeT. It subscribes to the topic 
        //subscribeTopicName and has the same queue size, queueSize. The &DoublePublisherSingleSubscriber::subscriberCallback,this is the syntax used to
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
    ros::Publisher publisherObject;
    ros::Publisher secondPublisherObject;
    //Then the node handle is declared
    /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
    ros::NodeHandle nH;        
};

#endif