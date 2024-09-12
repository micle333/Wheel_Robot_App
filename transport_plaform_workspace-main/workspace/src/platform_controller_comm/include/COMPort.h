#pragma once

#include <sys/ioctl.h>
#include <termios.h>
#include <cerrno>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <iostream>
#include <iomanip>
#include "utils.h"
#include <utility>
#include <vector>


class COMPort {
    public:
        COMPort(const std::string& port_name, int baud_rate);
        virtual ~COMPort();
        bool check_ready() const;
        bool get(std::vector<uint8_t> &message) const;
        void put(std::vector<uint8_t> message) const;
        bool empty() const;

    private:
        std::string port_name_;
        int baud_rate_;
        int32_t serial_;
        bool opened_ = false;
        long bytes_available_(int serial_tty) const;
        int32_t set_interface_attribs_(int32_t fd, int32_t speed) const;
    };

    COMPort::COMPort(const std::string& port_name, int baud_rate) {
        port_name_ = port_name;
        baud_rate_ = baud_rate;

        int32_t attempt = 1;
        serial_ = -1;

        while (serial_ < 0) {
            std::cout << "Trying to open: " << port_name_ << std::endl;
            serial_ = open(port_name_.c_str(), O_RDWR | O_SYNC);
            if (serial_ < 0){
                std::cerr << "Failed to open port:" << port_name_
                          << " with error code: " << std::string(strerror(errno)) << std::endl;
            }
            utils::delay(1000);
            ++attempt;
            if (attempt > 2) {
                return;
            }
        }
        if (set_interface_attribs_(serial_, baud_rate_) < 0){
            std::cerr << "Failed to configure port:" << port_name_ << std::endl;
            return;
        }
        opened_ = true;
        std::cout << "Port opened!" << std::endl;
    }

    COMPort::~COMPort() {
        close(serial_);
    }

    bool COMPort::check_ready() const {
        return opened_;
    }

    bool COMPort::get(std::vector<uint8_t> &message) const {
        long n = bytes_available_(serial_);
        if (n > 0) {
            message.resize(n);
            n = read(serial_, message.data(), n);
            if (n > 0) {
                message.resize(n);
                return true;
            }
        }
        return false;
    }

    void COMPort::put(std::vector<uint8_t> message) const {
        write(serial_, message.data(), message.size());
    }

    bool COMPort::empty() const {
        return 0 == bytes_available_(serial_);
    }

    long COMPort::bytes_available_(int serial_tty) const {
        int cnt = 0;
        ioctl(serial_tty, FIONREAD, &cnt);
        return long(cnt);
    }

    int32_t COMPort::set_interface_attribs_(int32_t fd, int32_t speed) const {
        struct termios tty;

        if (tcgetattr(fd, &tty) < 0) {
            std::cerr << "Error from tcgetattr: " << std::string(strerror(errno)) << std::endl;
            return -1;
        }

        cfsetospeed(&tty, (speed_t)speed);
        cfsetispeed(&tty, (speed_t)speed);

        tty.c_cflag |= (CLOCAL | CREAD);    /* ignore modem controls */
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;         /* 8-bit characters */
        tty.c_cflag &= ~PARENB;     /* no parity bit */
        tty.c_cflag &= ~CSTOPB;     /* only need 1 stop bit */
        tty.c_cflag &= ~CRTSCTS;    /* no hardware flowcontrol */

        /* setup for non-canonical mode */
        tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
        tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
        tty.c_oflag &= ~OPOST;

        /* fetch bytes as they become available */
        tty.c_cc[VMIN] = 1;
        tty.c_cc[VTIME] = 1;

        if (tcsetattr(fd, TCSANOW, &tty) != 0) {
            std::cerr << "Error from tcsetattr: " << std::string(strerror(errno)) << std::endl;
            return -1;
        }
        return 0;
    }
