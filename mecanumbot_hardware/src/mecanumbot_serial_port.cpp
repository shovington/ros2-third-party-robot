
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <rclcpp/rclcpp.hpp>

#include <fstream>
#include <nlohmann/json.hpp>
using json = nlohmann::json;

#include "mecanumbot_hardware/mecanumbot_serial_port.hpp"

#define HDLC_FRAME_BOUNDRY_FLAG     0x7E
#define HDLC_ESCAPE_FLAG            0x7D
#define HDLC_ESCAPE_XOR             0x20
#define HDLC_CRC_INIT_VALUE         0xFFFF

using namespace debict::mecanumbot::hardware;

MecanumbotSerialPort::MecanumbotSerialPort()
    : serial_port_(-1)
    , rx_frame_length_(0)
    , rx_frame_crc_(HDLC_CRC_INIT_VALUE)
    , rx_frame_escape_(false)
    , tx_frame_length_(0)
    , tx_frame_crc_(HDLC_CRC_INIT_VALUE)
{

}

MecanumbotSerialPort::~MecanumbotSerialPort()
{
    close();
}

return_type MecanumbotSerialPort::open(const std::string & port_name)
{
    serial_port_ = ::open(port_name.c_str(), O_RDWR | O_NOCTTY);

    if (serial_port_ < 0) {
        fprintf(stderr, "Failed to open serial port: %s (%d)\n", strerror(errno), errno);
        return return_type::ERROR;
    }

    struct termios tty_config{};
    if (::tcgetattr(serial_port_, &tty_config) != 0) {
        fprintf(stderr, "Failed to get serial port configuration: %s (%d)\n", strerror(errno), errno);
        close();
        return return_type::ERROR;
    }

    memset(&tty_config, 0, sizeof(termios));
    tty_config.c_cflag = B115200 | CRTSCTS | CS8 | CLOCAL | CREAD;
    tty_config.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
    tty_config.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
    tty_config.c_cflag &= ~CSIZE;    // Clear bits per byte
    tty_config.c_cflag |=  CS8;      // 8 bit per byte
    tty_config.c_iflag = IGNPAR;
    tty_config.c_oflag = OPOST;
    tty_config.c_lflag = 0;
    tty_config.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty_config.c_cc[VMIN] = 0;
    tcflush(serial_port_, TCIFLUSH);

    /*
    if (::cfsetispeed(&tty_config, B9600) != 0 || ::cfsetospeed(&tty_config, B9600) != 0) {
        fprintf(stderr, "Failed to set serial port speed: %s (%d)\n", strerror(errno), errno);
        close();
        return return_type::ERROR;
    }
    */

    if (::tcsetattr(serial_port_, TCSANOW, &tty_config) != 0) {
        fprintf(stderr, "Failed to set serial port configuration: %s (%d)\n", strerror(errno), errno);
        close();
        return return_type::ERROR;
    }

    return return_type::SUCCESS;
}

return_type MecanumbotSerialPort::close()
{
    if (is_open()) {
        ::close(serial_port_);
        serial_port_ = -1;
    }
    return return_type::SUCCESS;
}

return_type MecanumbotSerialPort::read_frames(char* buffer)
{
    // char buffer[1024];
    int buffer_index = 0;
    char last_char;
    
    while(last_char != '\n')
    {
        char buffer_[1024];
        ssize_t length = read(serial_port_, &buffer_, sizeof(buffer_));
        if (length == -1)
        {
            printf("Error reading from serial port\n");
            break;
        }

        for (int i = 0; i < length; i++)
        {
            buffer[buffer_index] = buffer_[i];
            buffer_index++;
        }

        last_char = buffer[buffer_index - 1];

    }

    // remrove carriage return and line feed
    buffer[buffer_index - 2] = '\0';
    std::cout << "buffer: " << buffer << std::endl;
    return return_type::SUCCESS;
}

return_type MecanumbotSerialPort::write_frame(char* data)
{
    // write to serial port
    ssize_t length = write(serial_port_, data, strlen(data));
    if (length == -1)
    {
        printf("Error writing to serial port\n");
    }

    return return_type::SUCCESS;
}

bool MecanumbotSerialPort::is_open() const
{
    return serial_port_ >= 0;
}