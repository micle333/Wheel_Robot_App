#include "ros/ros.h"
#include "platform_controller_comm/PacketContent.h"
#include "COM_bpnp_communicator.h"
#include <sstream>
#include <iomanip>


int main(int argc, char **argv) {
    ros::init(argc, argv, "central_hub_communication_node");
    ros::NodeHandle nh;

    auto ports = getSerialPorts();
    int baud_rate;

    COMBpnp port(ports, 115200, CMC_ID);
    if (!port.initializePort()) return -1;

    ros::Publisher pub = nh.advertise<platform_controller_comm::PacketContent>("central_hub_extracted_data", 1000);
    port.publisher = pub;
    port.subscriber = nh.subscribe<platform_controller_comm::PacketContent>("central_hub_command_data", 1000, &COMBpnp::packetCallback, &port);

    ros::Rate loop_rate(1000);
    while (ros::ok()) {
        port.processMessages();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
