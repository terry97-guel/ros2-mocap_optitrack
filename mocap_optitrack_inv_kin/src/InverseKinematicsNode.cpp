#include <InverseKinematicsNode.h>
#include <stdio.h>

//Class constructor
InverseKinematicsNode::InverseKinematicsNode(): Node("inverse_kinematics")
{
    //Declare the parameters of the node
    this->declare_parameter<int>("base_id", 0);
    this->declare_parameter<std::string>("sub_topic", "rigid_body_baseframe_topic");
    this->declare_parameter<std::string>("pub_topic", "configuration_topic");

    //Subscribe to the topic for RigidBody messages
    std::string sub_topic_;
    this->get_parameter("sub_topic", sub_topic_);
    char* sub_topic = (char*) malloc(sub_topic_.length()*sizeof(char));
    strcpy(sub_topic, sub_topic_.c_str());
    this->subscription_ = this->create_subscription<mocap_optitrack_interfaces::msg::RigidBodyArray>(
    sub_topic, 10, std::bind(&InverseKinematicsNode::rigid_body_topic_callback, this, _1));
  
    //Publisher definition
    std::string pub_topic_;
    this->get_parameter("pub_topic", pub_topic_);
    char* pub_topic = (char*) malloc(pub_topic_.length()*sizeof(char));
    strcpy(pub_topic, pub_topic_.c_str());
    this->publisher_ = this->create_publisher<mocap_optitrack_interfaces::msg::ConfigurationArray>(pub_topic, 10);

    //Create the node responsible of handling the inverse kinematics
    this->ik = InverseKinematics();
}

//Topic to receive the message of rigid bodies
void InverseKinematicsNode::rigid_body_topic_callback(const mocap_optitrack_interfaces::msg::RigidBodyArray::SharedPtr msg) const
{
    printf("Receceived message on rigid bodies....\n");
}

int main(int argc, char ** argv)
{
    //Create the node
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<InverseKinematicsNode>());
    rclcpp::shutdown();
    return 0;
}