#include "teleop-exo-suit/Serial.h"
#include <ros/ros.h>
#include <stdio.h>
#include <chrono>

using namespace std::chrono_literals;

void Serial::init(){
    try{
        this->serial = std::make_unique<asio::serial_port>(this->io, this->port);
        this->serial->set_option(asio::serial_port_base::baud_rate(this->baudrate));
        this->serial->set_option(asio::serial_port_base::character_size(8));
        this->serial->set_option(asio::serial_port_base::parity(asio::serial_port_base::parity::none));
        this->serial->set_option(asio::serial_port_base::stop_bits(asio::serial_port_base::stop_bits::one));
        this->serial->set_option(asio::serial_port_base::flow_control(asio::serial_port_base::flow_control::none));

        this->timer_ = std::make_unique<asio::steady_timer>(this->io);
        this->isConnect = true;
    }
    catch (const std::exception& e){
        this->isConnect = false;
        ROS_ERROR(e.what());
    }
}

void Serial::write(uint8_t* data_send, size_t data_size){
    system::error_code error;
    asio::write(*this->serial, asio::buffer(data_send, data_size), error);

}

DataRecv Serial::recv(){
    system::error_code error;

    DataRecv data_recv;

    data_recv.bytes_read = this->serial->read_some(asio::buffer(data_recv.data_recv), error);

    if(error){
        fmt::print(error.message()+'\n');
        return DataRecv();
    }

    return data_recv;
}

DataRecv Serial::recvWithTimeout(uint16_t milis) {
    DataRecv data_recv;
    system::error_code timer_error;
    system::error_code read_error;

    auto handler = [&](const system::error_code& error) {
        timer_error = error;
        this->serial->cancel(); 
    };

    this->timer_->expires_after( std::chrono::milliseconds(milis) );
    this->timer_->async_wait(handler);
    
    asio::async_read(*this->serial, asio::buffer(data_recv.data_recv),
                        [&](const system::error_code& error, std::size_t bytes_read) {
                            data_recv.bytes_read = bytes_read;
                            read_error = error;
                        });
    
    this->io.reset();
    while (this->io.run_one()){
        if (!read_error) {
            this->timer_->cancel();
        }
    }

    if(read_error.value() != 995){
        //fmt::println("{0} {1}",read_error.message(), read_error.value());
        this->isConnect = false;
    }
    
    if(!timer_error){
        return DataRecv();
    }
    
    return data_recv;
}