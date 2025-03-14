#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>

#include "teleop-exo-suit/AMT21.h"

void read_device_positions(AMT21& device, uint8_t* addrs, float* storage, int num_addrs){
    for(int i=0; i<num_addrs; ++i){
        ROS_INFO("iterate %d", i);
        device.set_address( addrs[i] );
        // storage[i] = device.read_position();
        storage[i] = i;
        ROS_INFO("iterate storage val %f", storage[i] );
    }
}

void publish_device_position(ros::Publisher& publisher, float* storage, int num_storage){
    std_msgs::Float32MultiArray data;

    data.data.resize( num_storage );
    for(int i=0; i<num_storage; ++i){
        data.data[i] = storage[i];
    }
    publisher.publish(data);

}

int main(int argc, char** argv){
    ros::init(argc, argv, "amt21_publisher");
    ros::NodeHandle nh;
    ros::Rate loop_rate(1000);

    ros::Publisher amt21_publisher = nh.advertise<std_msgs::Float32MultiArray>(
        "/amt21/positions", 
        10
    );

    int num_joints = 0;
    if(nh.hasParam("/num_joints")) nh.getParam("/num_joints", num_joints);

    uint8_t list_device_addrs[num_joints]     = {0};
    float   list_device_positions[num_joints] = {0};
    for(int i=0; i<num_joints; ++i){
        int addrs;
        nh.getParam(fmt::format("/amt{}", i), addrs);
        list_device_addrs[i] = (uint8_t)addrs;
        ROS_INFO("device addr [%d] %d", i, addrs);
    }
    
    AMT21 amt21("/dev/ttyACM0",
        115200,
        list_device_addrs[0]
    );
    amt21.init();

    if(amt21.isConnect){
        while(ros::ok()){

            read_device_positions(amt21, list_device_addrs, list_device_positions, num_joints);
            publish_device_position(amt21_publisher, list_device_positions, num_joints);
            
            ros::spinOnce();
            loop_rate.sleep();
        }
    }

    return 0;
}