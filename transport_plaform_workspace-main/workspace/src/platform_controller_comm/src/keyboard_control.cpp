#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <iomanip>
#include <termios.h>
#include <geometry_msgs/TwistStamped.h>
#include <actionlib/client/simple_action_client.h>
#include "platform_controller_comm/WheelRotation.h"
#include "platform_controller_comm/LightCommand.h"
#include "platform_controller_comm/MotorsServices.h"
#include "platform_controller_comm/MotorsActionAction.h"

#define KEYCODE_SPACE 0x20
#define KEYCODE_UP 0x38           // 8
#define KEYCODE_DOWN 0x35         // 5
#define KEYCODE_RIGHT 0x36        // 6
#define KEYCODE_LEFT 0x34         // 4
#define KEYCODE_CALIBRATE_FRONT_AXIS 0x37        // 7
#define KEYCODE_CALIBRATE_REAR_AXIS 0x39         // 9

#define KEYCODE_ESC 0x1b
#define KEYCODE_EXIT 0x60             // `
#define KEYCODE_LEFT_TURN_ON 0x71     // Q
#define KEYCODE_LEFT_TURN_OFF 0x61    // A
#define KEYCODE_STOP_ON 0x77          // W
#define KEYCODE_STOP_OFF 0x73         // S
#define KEYCODE_RIGHT_TURN_ON 0x65    // E
#define KEYCODE_RIGHT_TURN_OFF 0x64   // D

#define KEYCODE_FRONT_AXIS_RIGHT_TURN 0x5D  // [
#define KEYCODE_FRONT_AXIS_LEFT_TURN 0x5B   // ]
#define KEYCODE_REAR_AXIS_RIGHT_TURN 0x2E   // <
#define KEYCODE_REAR_AXIS_LEFT_TURN 0x2C    // >

#define KEYCODE_HANDBRAKE 0x7F             // Backspace

#define AXIS_ANGLE_LIMIT 20
#define WHEEL_SPEED_STEP 0.5
#define WHEEL_ANGLE_STEP 5

typedef actionlib::SimpleActionClient<platform_controller_comm::MotorsActionAction> Client;


int main (int argc, char** argv){
    // ROS node and topics initialization
    ros::init(argc, argv, "keyboard_control");
    ros::NodeHandle nh;
    ros::ServiceClient lightControlClient = nh.serviceClient<platform_controller_comm::LightCommand>("light_command");
    ros::ServiceClient handbrakeControlClient = nh.serviceClient<platform_controller_comm::MotorsServices>("motors_services");
    Client client("motors_actions", true);
    client.waitForServer();
    ros::Publisher wheelDataPub = nh.advertise<platform_controller_comm::WheelRotation>("manual_control", 2);
    float currentVelocity = 0.0;
    float currentFrontAxisTurnAngle = 0.0;
    float currentRearAxisTurnAngle = 0.0;
    bool handbrakeStatus = false;
    platform_controller_comm::LightCommand lightCommand;
    platform_controller_comm::MotorsServices motorsServices;
    platform_controller_comm::MotorsActionGoal goal;

    int kfd = 0;
    struct termios cooked, raw;
    char c;
    bool commandSet = false;

    // Non-blocking terminal key reading part
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);

    puts("Reading from keyboard");
    puts("---------------------------");
    puts("Light controls.");
    puts("W - Stop On");
    puts("S - Stop Off");
    puts("Q - Left turn On");
    puts("A - Left turn Off");
    puts("E - Right turn On");
    puts("D - Right turn Off");
    puts("---------------------------");
    puts("Use arrow keys to move the robot.");
    puts("8 - +0.5 m/s speed");
    puts("5 - -0.5 m/s speed");
    puts("4 - -5 degrees turn angle");
    puts("6 - +5 degrees turn angle");
    puts("7 - front axis calibrate");
    puts("9 - rear axis calibrate");
    puts("Space - stop the robot");
    puts("[ - turn front axis left");
    puts("] - turn front axis right");
    puts("< - turn rear axis left");
    puts("> - turn rear axis left");
    puts("Backspace - hanbrake on/off");
    puts("---------------------------");
    puts("` - Exit");

    while (ros::ok())
    {
      // read a key press event
      if(read(kfd, &c, 1) < 0)
      {
        perror("read():");
        exit(-1);
        return -1;
      }
      lightCommand.request.header.stamp = ros::Time::now();
      commandSet = true;
      switch(c)
      {
          case KEYCODE_LEFT_TURN_ON:
            ROS_INFO_STREAM("LEFT TURN LIGHT ON");
            lightCommand.request.cmd = "Left turn On";
            lightControlClient.call(lightCommand);
            break;
          case KEYCODE_LEFT_TURN_OFF:
            ROS_INFO_STREAM("LEFT TURN LIGHT OFF");
            lightCommand.request.cmd = "Left turn Off";
            lightControlClient.call(lightCommand);
            break;
          case KEYCODE_STOP_ON:
            ROS_INFO_STREAM("STOP LIGHT ON");
            lightCommand.request.cmd = "Stop On";
            lightControlClient.call(lightCommand);
            break;
          case KEYCODE_STOP_OFF:
            ROS_INFO_STREAM("STOP LIGHT OFF");
            lightCommand.request.cmd = "Stop Off";
            lightControlClient.call(lightCommand);
            break;
          case KEYCODE_RIGHT_TURN_ON:
            ROS_INFO_STREAM("RIGHT TURN LIGHT ON");
            lightCommand.request.cmd = "Right turn On";
            lightControlClient.call(lightCommand);
            break;
          case KEYCODE_RIGHT_TURN_OFF:
            ROS_INFO_STREAM("RIGHT TURN LIGHT OFF");
            lightCommand.request.cmd = "Right turn Off";
            lightControlClient.call(lightCommand);
            break;
          case KEYCODE_SPACE:
            ROS_INFO_STREAM("SPACE");
            currentVelocity = 0.0;
            currentFrontAxisTurnAngle = 0.0;
            currentRearAxisTurnAngle = 0.0;
            break;
          case KEYCODE_UP:
            ROS_INFO_STREAM("UP");
            currentVelocity += WHEEL_SPEED_STEP;
            break;
          case KEYCODE_DOWN:
            ROS_INFO_STREAM("DOWN");
            currentVelocity -= WHEEL_SPEED_STEP;
            break;
          case KEYCODE_RIGHT:
            ROS_INFO_STREAM("RIGHT");
            if (currentFrontAxisTurnAngle>-AXIS_ANGLE_LIMIT) {
              currentFrontAxisTurnAngle -= WHEEL_ANGLE_STEP;
            }
            if (currentRearAxisTurnAngle<AXIS_ANGLE_LIMIT) {
              currentRearAxisTurnAngle += WHEEL_ANGLE_STEP;
            }
            break;
          case KEYCODE_LEFT:
            ROS_INFO_STREAM("LEFT");
            if (currentFrontAxisTurnAngle<AXIS_ANGLE_LIMIT) {
              currentFrontAxisTurnAngle += WHEEL_ANGLE_STEP;
            }
            if (currentRearAxisTurnAngle>-AXIS_ANGLE_LIMIT) {
              currentRearAxisTurnAngle -= WHEEL_ANGLE_STEP;
            }
            break;
          case KEYCODE_FRONT_AXIS_RIGHT_TURN:
            ROS_INFO_STREAM("FRONT AXIS RIGHT");
            if (currentFrontAxisTurnAngle>-AXIS_ANGLE_LIMIT) {
              currentFrontAxisTurnAngle -= WHEEL_ANGLE_STEP;
            }
            break;
          case KEYCODE_FRONT_AXIS_LEFT_TURN:
            ROS_INFO_STREAM("FRONT AXIS LEFT");
            if (currentFrontAxisTurnAngle<AXIS_ANGLE_LIMIT) {
              currentFrontAxisTurnAngle += WHEEL_ANGLE_STEP;
            }
            break;
          case KEYCODE_REAR_AXIS_RIGHT_TURN:
            ROS_INFO_STREAM("REAR AXIS RIGHT");
            if (currentRearAxisTurnAngle<AXIS_ANGLE_LIMIT) {
              currentRearAxisTurnAngle += WHEEL_ANGLE_STEP;
            }
            break;
          case KEYCODE_REAR_AXIS_LEFT_TURN:
            ROS_INFO_STREAM("REAR AXIS LEFT");
            if (currentRearAxisTurnAngle>-AXIS_ANGLE_LIMIT) {
              currentRearAxisTurnAngle -= WHEEL_ANGLE_STEP;
            }
            break;
          case KEYCODE_HANDBRAKE:
            bool call;
            if (handbrakeStatus) {
              ROS_INFO_STREAM("HANDBRAKE FREE");
              motorsServices.request.cmd = "FREE HANDBRAKE";
              call = handbrakeControlClient.call(motorsServices);
            } else {
              ROS_INFO_STREAM("HANDBRAKE");
              motorsServices.request.cmd = "BRAKE";
              currentVelocity = 0.0;
              currentRearAxisTurnAngle = 0.0;
              currentFrontAxisTurnAngle = 0.0;
              call = handbrakeControlClient.call(motorsServices);
            }
            if (call) {
              handbrakeStatus = motorsServices.response.result;
            }
            // std::cout << handbrakeStatus << std::endl;
            break;
          case KEYCODE_CALIBRATE_FRONT_AXIS:
            ROS_INFO_STREAM("CALIBRATE_FRONT_AXIS");
            goal.cmd = "FRONT AXIS CALIBRATION";
            currentVelocity = 0.0;
            currentRearAxisTurnAngle = 0.0;
            currentFrontAxisTurnAngle = 0.0;
            client.sendGoal(goal);
            break;
          case KEYCODE_CALIBRATE_REAR_AXIS:
            ROS_INFO_STREAM("CALIBRATE_REAR_AXIS");
            goal.cmd = "REAR AXIS CALIBRATION";
            currentVelocity = 0.0;
            currentRearAxisTurnAngle = 0.0;
            currentFrontAxisTurnAngle = 0.0;
            client.sendGoal(goal);
            break;
          case KEYCODE_EXIT:
            {
              // stop reading keys
              tcsetattr(STDIN_FILENO, TCSANOW, &cooked);
              commandSet = false;
              platform_controller_comm::WheelRotation msg;

              msg.header.stamp = ros::Time::now();
              msg.frontLeft = 0;
              msg.frontRight = 0;
              msg.rearLeft = 0;
              msg.rearRight = 0;
              msg.rearAxisTargetAngle = 0;
              msg.frontAxisTargetAngle = 0;

              wheelDataPub.publish(msg);

              exit(1);
              break;
            }
          default:
            std::cout << "Hex: " << std::hex << std::uppercase << std::setw(2) << std::setfill('0') << static_cast<int>(static_cast<unsigned char>(c)) << std::endl;
            commandSet = false;
            break;
        }
        if (commandSet) {
          platform_controller_comm::WheelRotation msg;

          msg.header.stamp = ros::Time::now();
          if (currentVelocity > 10) {
            currentVelocity = 10;
          }
          if (currentVelocity < -10) {
            currentVelocity = -10;
          }
          msg.frontLeft = currentVelocity;
          msg.frontRight = currentVelocity;
          msg.rearLeft = currentVelocity;
          msg.rearRight = currentVelocity;
          msg.rearAxisTargetAngle = currentRearAxisTurnAngle;
          msg.frontAxisTargetAngle = currentFrontAxisTurnAngle;

          wheelDataPub.publish(msg);
        }
    }
    // return terminal window to normal state
    tcsetattr(STDIN_FILENO, TCSANOW, &cooked);
}
