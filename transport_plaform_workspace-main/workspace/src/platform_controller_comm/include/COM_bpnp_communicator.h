#pragma once

#include <iostream>
#include <vector>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <stdexcept>
#include <string>
#include <dirent.h>
#include "ros/ros.h"
#include "platform_controller_comm/PacketContent.h"
#include "utils.h"
#include "COMPort.h"
#include "bpnp_packet_detector.h"
#include "bpnp_packet_packer.h"

std::vector<std::string> getSerialPorts() {
    std::vector<std::string> ports;
    const char* devDir = "/dev";
    DIR* dir = opendir(devDir);

    if (dir != nullptr) {
        struct dirent* ent;

        while ((ent = readdir(dir)) != nullptr) {
            std::string devName = ent->d_name;

            // Фильтрация устройств, имена которых начинаются с ttyS (стандартные COM порты) и ttyUSB (USB-to-serial)
            if (devName.find("ttyUSB") == 0 || devName.find("ttyACM") == 0) {
                ports.push_back(std::string(devDir) + "/" + devName);
            }
        }
        closedir(dir);
    } else {
        std::cerr << "Failed to open directory " << devDir << std::endl;
    }

    return ports;
}

class COMBpnp {
public:
    COMBpnp(const std::vector<std::string>& ports, int baudRate, int deviceId)
        : baudRate_(baudRate), ports_(ports), targetDevice_(deviceId) {}

    ros::Publisher publisher;
    ros::Subscriber subscriber;

    // Проверка всех портов и выбор первого доступного
    bool initializePort() {
        for (auto& portName : ports_) {
            COMPort port(portName, baudRate_);
            if (checkPort(port)) {
                std::cout << "Port " << portName << " is operational." << std::endl;
                activePort_ = std::make_unique<COMPort>(portName, baudRate_);
                return true;
            } else {
                std::cout << "Port " << portName << " failed to initialize." << std::endl;
            }
        }
        return false;
    }

    void packetCallback(const platform_controller_comm::PacketContent::ConstPtr& msg) {
        if (!activePort_) {
            std::cerr << "No operational port available." << std::endl;
            return;
        }
        std::vector<uint8_t> packedData = packMessage(msg->data);
        activePort_->put(packedData);
    }

    void processMessages() {
        if (!activePort_) {
            std::cerr << "No operational port available." << std::endl;
            return;
        }
        if (!activePort_->empty()) {
            std::vector<uint8_t> message;
            activePort_->get(message);
            for (auto incomingByte : message) {
                uint8_t parsingResult = packetFinder_.processByte(incomingByte);
                if (parsingResult == PacketFinder::DONE) {
                    platform_controller_comm::PacketContent msg;
                    msg.header.stamp = ros::Time::now();
                    msg.pack_name = packetFinder_.getPackNameByID(packetFinder_.getPacketId());
                    size_t dataSize = packetFinder_.getPacketDataLength();
                    std::vector<uint8_t> data = {packetFinder_.getPacketId(), packetFinder_.getSenderId()};
                    const uint8_t* dataPointer = packetFinder_.getPacketData();
                    data.insert(data.end(), dataPointer, dataPointer + dataSize);
                    msg.data = data;
                    publisher.publish(msg);
                }
            }
        }
    }

private:
    std::vector<std::string> ports_;
    int baudRate_;
    int targetDevice_;
    std::unique_ptr<COMPort> activePort_;
    PacketFinder packetFinder_;


    // Функция для отправки тестового пакета и проверки ответа
    bool checkPort(COMPort& port) {
        std::cout << "Checking port opened..." << std::endl;
        if (!port.check_ready()) return false;

        std::mutex mtx;
        std::condition_variable cv;
        bool completed = false;

        auto check = [&]() {
            try {
                std::vector<uint8_t> data;
                sendWHUPacket(port);
                utils::delay(100);
                port.get(data);
                if (!data.empty()) {
                    std::lock_guard<std::mutex> lock(mtx);
                    check_ = checkRecievedData(data);
                }
                cv.notify_one();
            } catch (std::exception& e) {
                std::lock_guard<std::mutex> lock(mtx);
                check_ = false;
                cv.notify_one();
            }
            completed = true;
        };

        std::thread worker(check);
        std::unique_lock<std::mutex> lock(mtx);
        if (!cv.wait_for(lock, std::chrono::seconds(5), [&completed] { return completed; })) {
            if (worker.joinable()) worker.detach();
            return false;
        }
        if (worker.joinable()) worker.join();
        return check_;
    }

    void sendWHUPacket(COMPort& port) {
        std::vector<uint8_t> payload = {WHU_ID, BRT_ID};
        auto packet = packMessage(payload);
        port.put(packet);
    }

    bool checkRecievedData(const std::vector<uint8_t>& data) {
        PacketFinder packetFinder;
        for (auto incomingByte : data) {
            uint8_t parsingResult = packetFinder.processByte(incomingByte);
            if (parsingResult == PacketFinder::DONE) {
                uint8_t packetID = packetFinder.getPacketId();
                std::cout << "Got answer" << std::endl;
                return packetFinder.getSenderId() == targetDevice_;
            }
        }
        return false;
    }

    bool check_ = false;
};
