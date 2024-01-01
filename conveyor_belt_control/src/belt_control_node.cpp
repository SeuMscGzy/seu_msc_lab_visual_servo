#include <libserial/SerialPort.h>
#include <libserial/SerialStream.h>
#include <iostream>
#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"

constexpr const char* const SERIAL_PORT_2 = "/dev/ttyUSB0";
int state = 0;

class MotorController : public rclcpp::Node
{
public:
    MotorController()
    : Node("motor_controller")
    {
        subscriber_ = this->create_subscription<std_msgs::msg::Int64>(
            "motor_state", 10,
            std::bind(&MotorController::motor_state_callback, this, std::placeholders::_1));
        my_serial_port_.Open(SERIAL_PORT_2);
        std::cout<<my_serial_port_.IsOpen()<<std::endl;
        my_serial_port_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
        my_serial_port_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
        my_serial_port_.SetParity(LibSerial::Parity::PARITY_NONE);
        my_serial_port_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
        my_serial_port_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
    }

    void motor_state_callback(const std_msgs::msg::Int64::SharedPtr msg)
    {
        state = msg->data;
    }

    void run()
    {
        rclcpp::Rate rate(50);

        while (rclcpp::ok())
        {
            // Update the data vector based on the state
            update_data_vector(state);

            // Write to the serial port
            my_serial_port_.Write(data_);

            // Sleep for a while
            rate.sleep();

            // Spin some callbacks
            rclcpp::spin_some(shared_from_this());
        }
    }

private:
    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr subscriber_;
    LibSerial::SerialPort my_serial_port_;
    std::vector<uint8_t> data_;

    void update_data_vector(int state)
    {
        switch (state)
        {
            case -1:
                data_ = {0x55, 0xaa, 0x07, 0x10, 0x27, 0x00, 0x00, 0x00, 0x00, 0xc3};
                break;
            case -2:
                data_ = {0x55, 0xaa, 0x07, 0x20, 0x4e, 0x00, 0x00, 0x00, 0x00, 0xc3};
                break;
            case -3:
                data_ = {0x55, 0xaa, 0x07, 0x30, 0x75, 0x00, 0x00, 0x00, 0x00, 0xc3};
                break;
            case -4:
                data_ = {0x55, 0xaa, 0x07, 0x40, 0x9c, 0x00, 0x00, 0x00, 0x00, 0xc3};
                break;
            case 0:
                data_ = {0x55, 0xaa, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc3};
                break;
            case 1:
                data_ = {0x55, 0xaa, 0x07, 0x10, 0x27, 0x90, 0x7d, 0xfc, 0xff, 0xc3};
                break;
            case 2:
                data_ = {0x55, 0xaa, 0x07, 0x20, 0x4e, 0x90, 0x7d, 0xfc, 0xff, 0xc3};
                break;
            case 3:
                data_ = {0x55, 0xaa, 0x07, 0x30, 0x75, 0x90, 0x7d, 0xfc, 0xff, 0xc3};
                break;
            case 4:
                data_ = {0x55, 0xaa, 0x07, 0x40, 0x9c, 0x90, 0x7d, 0xfc, 0xff, 0xc3};
                break;
            default:
                // Handle any other states if necessary
                break;
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto motor_controller = std::make_shared<MotorController>();
    motor_controller->run();
    rclcpp::shutdown();
    return 0;
}