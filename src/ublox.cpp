#include "UBLOX/ublox.h"

UBLOX::UBLOX(rtk_type_t type, std::string port) :
    serial_(port, 115200),
    ubx_(serial_),
    type_(type)
{
    // configure the parsers
    ubx_.set_nav_rate(100);
    ubx_.enable_message(UBX::CLASS_NAV, UBX::NAV_PVT, 10);
    ubx_.enable_message(UBX::CLASS_NAV, UBX::NAV_POSECEF, 10);
    ubx_.enable_message(UBX::CLASS_NAV, UBX::NAV_VELECEF, 10);
    ubx_.enable_message(UBX::CLASS_CFG, UBX::CFG_VALGET, 10);

    // configure RTCM message
    if (type_ & RTK == RTK )
        ubx_.turnOnRTCM();
    if (type_ == ROVER)
        configRover();
    if (type_ == BASE)
        configBase();
}

UBLOX::~UBLOX()
{
    if (udp_)
        delete udp_;
}

void UBLOX::serial_read_cb(const uint8_t* buf, size_t size)
{

}

void UBLOX::udp_read_cb(const uint8_t* buf, size_t size)
{
    assert(type_ == ROVER);

    for (int i = 0; i < size; i++)
        rtcm_.read_cb(buf[i]);
}


void UBLOX::configRover()
{
    if (udp_)
        throw std::runtime_error("Unable to create two UDP connections");

    // hook up UDP to listen
    udp_ = new async_comm::UDP("localhost", 14620, "localhost", 14625);
    auto rec_cb_trampoline = [this](const uint8_t* buf, size_t size)
    {
        this->udp_read_cb(buf, size);
    };
    udp_->register_receive_callback(rec_cb_trampoline);

    if (!udp_->init())
    {
        throw std::runtime_error("Failed to initialize Rover receive UDP");
    }
}

void UBLOX::configBase()
{
    if (udp_)
        throw std::runtime_error("Unable to create two UDP connections");

    // hook up UDP to send
    udp_ = new async_comm::UDP("localhost", 14625, "localhost", 14620);

    if (!udp_->init())
    {
        throw std::runtime_error("Failed to initialize Rover receive UDP");
    }
}

void UBLOX::update()
{
    rtcm_.msg_lock_.lock();
    if (rtcm_.new_data())
    {
        serial_.send_bytes(rtcm_.out_buffer_, rtcm_.message_len_);
    }
    rtcm_.msg_lock_.unlock();


}


