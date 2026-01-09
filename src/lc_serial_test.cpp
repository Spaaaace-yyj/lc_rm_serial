#include "../include/lc_serial_test/lc_serial_test.hpp"

LcSerialTestNode::LcSerialTestNode(const rclcpp::NodeOptions & options) 
        : Node("serial_test",  options)
        , owned_ctx_{ new IoContext(2) }
        , serial_driver_{ new drivers::serial_driver::SerialDriver(*owned_ctx_) }
{

    RCLCPP_INFO(this->get_logger(), "Starting serial node...");

    using FlowControl = drivers::serial_driver::FlowControl;
    using Parity = drivers::serial_driver::Parity;
    using StopBits = drivers::serial_driver::StopBits;

    uint32_t baud_rate = 921600;
    auto fc = FlowControl::NONE;
    auto pt = Parity::NONE;
    auto sb = StopBits::ONE;
    
    device_config_ = std::make_unique<drivers::serial_driver::SerialPortConfig>
        (baud_rate, fc, pt, sb);
    
    try{
        serial_driver_->init_port("/dev/ttyACM0", *device_config_);
        if(!serial_driver_->port()->is_open()){
            serial_driver_->port()->open();
            receive_thread_ = std::thread(&LcSerialTestNode::receiveLoop, this);
        }
    }catch (const std::exception& ex){
        RCLCPP_ERROR(rclcpp::get_logger("lc_serial"), "Error creating lc_serial port: %s - %s", device_name_.c_str(), ex.what());
        throw ex;
    }

}

void LcSerialTestNode::DecodeData(){
    uint16_t flags_register;
    get_protocol_info(buffer.data(), &flags_register, (uint8_t *)&recv_data.yaw);
    std::cout << "decode result:" << recv_data.yaw << ", " << recv_data.pitch << ", " << recv_data.row << std::endl;
}

void LcSerialTestNode::SendData(){
    
}

void LcSerialTestNode::receiveLoop()
{

    while (rclcpp::ok())
    {
        try
        {
            buffer.clear();
            buffer.resize(256);
            size_t n = serial_driver_->port()->receive(buffer);
            if(n <= 10){
                RCLCPP_WARN(this->get_logger(), "receive data is too short, skip this loop! [%d byte]", n);
                continue;
            };

            if(buffer[0] == 0xA5){
                std::cout << "receive [" << n << " byte] data : ";
                for (size_t i = 0; i < n; i++){
                    std::cout << std::hex
                    << std::setw(2)
                    << std::setfill('0')
                    << static_cast<int>(buffer[i])
                    << " ";
                }
                std::cout << std::dec << std::endl;
                DecodeData();
            }else{
                RCLCPP_WARN(this->get_logger(), "Can't find CMD_ID! skip this loop!");
            }

        }catch (const std::exception & e){
            RCLCPP_ERROR(this->get_logger(), "Serial read error: %s", e.what());
            break;
        }
    }
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LcSerialTestNode>(rclcpp::NodeOptions()));
    rclcpp::shutdown();
    return 0;
}
