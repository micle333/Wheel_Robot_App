#include "ros/ros.h"
#include "platform_controller_comm/PacketContent.h"
#include "COM_bpnp_communicator.h"
#include "message_waiter.h"
#include <sstream>
#include <iomanip>


int main(int argc, char **argv) {
    ros::init(argc, argv, "front_motor_communication_node");
    ros::NodeHandle nh;

    {
        MessageWaiter<platform_controller_comm::PacketContent> centralHubWaiter(nh, "central_hub_extracted_data", 10);
        centralHubWaiter.waitForMessage(0);
    }
    auto ports = getSerialPorts();
    int baud_rate;

    COMBpnp port(ports, 115200, FPB_ID);
    if (!port.initializePort()) return -1;

    ros::Publisher pub = nh.advertise<platform_controller_comm::PacketContent>("front_motor_extracted_data", 1000);
    port.publisher = pub;
    port.subscriber = nh.subscribe<platform_controller_comm::PacketContent>("front_motor_command_data", 1000, &COMBpnp::packetCallback, &port);

    ros::Rate loop_rate(1000);
    while (ros::ok()) {
        port.processMessages();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
