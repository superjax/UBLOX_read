#include "UBLOX/ublox.h"
#define DBG(...) fprintf(stderr, __VA_ARGS__)

namespace ublox
{

UBLOX::UBLOX(const std::string& port) :
    serial_(port, 921600),
    //115200
    ubx_(serial_)
{
    type_ = NONE;

    auto cb = [this](const uint8_t* buffer, size_t size)
    {
        this->serial_read_cb(buffer, size);
    };
    serial_.register_receive_callback(cb);
    serial_.init();

    // configure the parsers/Enable Messages
    ubx_.set_nav_rate(100);
    ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 1, CFG_VALSET_t::MSGOUT_PVT, byte);
    ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 1, CFG_VALSET_t::MSGOUT_RELPOSNED, byte);
    ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 1, CFG_VALSET_t::MSGOUT_POSECEF, byte);
    ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 1, CFG_VALSET_t::MSGOUT_VELECEF, byte);
    ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 0, CFG_VALSET_t::MSGOUT_RAWX, byte);
    ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 0, CFG_VALSET_t::MSGOUT_SFRBX, byte);
    //configuring SVIN messages is done in config_base_stationary()


    auto eph_cb = [this](uint8_t cls, uint8_t type, const ublox::UBX_message_t& in_msg)
    {
      this->nav_.convertUBX(in_msg.RXM_SFRBX);
    };
    ubx_.registerCallback(ublox::CLASS_RXM, ublox::RXM_SFRBX, eph_cb);
}

void UBLOX::config_f9p()
{
    ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, CFG_VALSET_t::DYNMODE_AIRBORNE_1G, CFG_VALSET_t::DYNMODEL, byte);
    ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 0, CFG_VALSET_t::USB_INPROT_NMEA, byte);
    ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 0, CFG_VALSET_t::USB_OUTPROT_NMEA, byte);

    bool poll = true;
    if(poll == true)
        poll_value();
}

/*
Function config_base 
Configures whether the base is stationary or moving.
Inputs: base type
Outputs: calls the correct base configuration function and outputs to the command line the status of the base.
*/
void UBLOX::config_base(const std::string base_type="stationary")
{
    //Choose to configure as moving/mobile base or stationary
    //bool mobile = false;
    if(base_type == "moving")   //Moving base
    {   
        config_base_moving(1);
        config_base_stationary(0);
        std::cerr<<"Moving Base\n";
    }
    else if(base_type == "stationary")  //Stationary base
    { 
        config_base_moving(0);
        config_base_stationary(1);
        std::cerr<<"Stationary Base\n";
    }
    else    //Error thrown when type of base is unrecognized
    {
        throw std::runtime_error("Failed to initialize base as moving or stationary");
    }
}

void UBLOX::config_base_stationary(int on_off)
{
    ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 1*on_off, CFG_VALSET_t::RTCM_1005USB, byte);
    ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 1*on_off, CFG_VALSET_t::RTCM_1074USB, byte);
    ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 1*on_off, CFG_VALSET_t::RTCM_1084USB, byte);
    ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 1*on_off, CFG_VALSET_t::RTCM_1094USB, byte);
    ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 1*on_off, CFG_VALSET_t::RTCM_1124USB, byte);
    ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 1, CFG_VALSET_t::RTCM_1230USB, byte);

    ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 1*on_off, CFG_VALSET_t::MSGOUT_SVIN, byte);
    ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 1*on_off, CFG_VALSET_t::TMODE_MODE, byte);
    ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 500000*on_off, CFG_VALSET_t::TMODE_SVIN_ACC_LIMIT, word);
    ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 119*on_off, CFG_VALSET_t::TMODE_SVIN_MIN_DUR, word);

}

void UBLOX::config_base_moving(int on_off)
{

    ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 1*on_off, CFG_VALSET_t::RTCM_4072_0USB, byte);
    ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 1*on_off, CFG_VALSET_t::RTCM_4072_1USB, byte);
    ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 1*on_off, CFG_VALSET_t::RTCM_1077USB, byte);
    ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 1*on_off, CFG_VALSET_t::RTCM_1087USB, byte);
    ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 1*on_off, CFG_VALSET_t::RTCM_1097USB, byte);
    ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 1*on_off, CFG_VALSET_t::RTCM_1127USB, byte);
    ubx_.configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 1, CFG_VALSET_t::RTCM_1230USB, byte);
}

void UBLOX::config_rover()
{
}

void UBLOX::poll_value()
{
    ubx_.get_configuration(CFG_VALGET_t::REQUEST, CFG_VALGET_t::RAM, CFG_VALGET_t::RTCM_1005USB);
    ubx_.get_configuration(CFG_VALGET_t::REQUEST, CFG_VALGET_t::RAM, CFG_VALGET_t::RTCM_1230USB);
    ubx_.get_configuration(CFG_VALGET_t::REQUEST, CFG_VALGET_t::RAM, CFG_VALGET_t::RTCM_1097USB);
}

void UBLOX::readFile(const std::string& filename)
{
    std::ifstream file(filename,  std::ifstream::binary);
    file.seekg(0, file.end);
    uint32_t len = file.tellg();
    file.seekg (0, file.beg);
    char* buffer = new char [len];
    file.read(buffer, len);
    int idx = 0;
    while(idx < len)
    {
        int chunk_size = ((len - idx) >= 14)?14:len-idx;
        serial_read_cb(((const uint8_t*)buffer)+idx, chunk_size);
        idx = idx+chunk_size;
        usleep(1000);
    }
    // serial_read_cb((const uint8_t*)buffer, len);
}

void UBLOX::initLogFile(const std::string& filename)
{
    if (log_file_.is_open())
        log_file_.close();

    log_file_.open(filename);
}

//Function initRover
//This function initializes a rover that is given by the following data.
void UBLOX::initRover(std::string local_host, uint16_t local_port,
                      std::string remote_host, uint16_t remote_port)
{
    type_ = ROVER;

    assert(udp_ == nullptr);
    // Connect the rtcm_cb callback to forward data to the UBX serial port
    rtcm_.registerCallback([this](uint8_t* buf, size_t size)
    {
        this->rtcm_complete_cb(buf, size);
    });

    // hook up UDP to listen
    /// TODO: configure ports and IP from cli
    udp_ = new async_comm::UDP(local_host, local_port, remote_host, remote_port);
    udp_->register_receive_callback([this](const uint8_t* buf, size_t size)
    {
        this->udp_read_cb(buf, size);
    });

    if (!udp_->init())
        throw std::runtime_error("Failed to initialize Rover receive UDP");

    config_rover();
    config_f9p();
}

void UBLOX::initBase(std::string local_host, uint16_t local_port,
                       std::string remote_host, uint16_t remote_port,
                       //std::string local_host2, uint16_t local_port2,
                       //std::string remote_host2, uint16_t remote_port2,
                       std::string base_type, int rover_quantity)
{
    type_ = BASE;

    //Create an array of UDP objects
    async_comm::UDP* udparray_[rover_quantity];

    //Fill udp objects into the array.
    for(int i = 0; i < rover_quantity; i++) {
        udparray_[i] = new async_comm::UDP(local_host, local_port, remote_host, remote_port);
    }

    

    if (udp_)
        throw std::runtime_error("Unable to create two UDP connections");

    //Make udp_ point to each object in the array in a for loop and initialize the connections
    udp_ = udparray_[0];

    // hook up UDP to send
    //udp_ is a pointer
    udp_ = new async_comm::UDP(local_host, local_port, remote_host, remote_port);

    //if udp_ does not point to an object that has an init() method, throw an error.
    //Otherwise initialize udp
    if (!udp_->init())
    {
        throw std::runtime_error("Failed to initialize Base to Rover 1 receive UDP");
    }

    rtcm_.registerCallback([this](uint8_t* buf, size_t size)
    {
        //std::cerr << "buf1 = " << buf << "\n";
        //std::cerr << "size1 = " << size << "\n";
        this->udp_->send_bytes(buf, size);
    });

/*/////////////////////////////////////////////////////////
    udp2_ = new async_comm::UDP(local_host2, local_port2, remote_host2, remote_port2);

    if (!udp2_->init())
    {
        throw std::runtime_error("Failed to initialize Base to Rover 2 receive UDP");
    }

    rtcm_.registerCallback([this](uint8_t* buf, size_t size)
    {
        //std::cerr << "buf2 = " << buf << "\n";
        //std::cerr << "size2 = " << size << "\n";
        // this->udp_->send_bytes(buf, size);
        this->udp2_->send_bytes(buf, size);
    });
/////////////////////////////////////*/
    config_base(base_type);
    config_f9p();
}

void UBLOX::initBase(std::string local_host[], int local_port[],
                    std::string remote_host[], int remote_port[],
                    std::string base_type, int rover_quantity)
{
    type_ = BASE;

    //Create an array of UDP objects
    async_comm::UDP* udparray_[rover_quantity];

    //Fill udp objects into the array.
    for(int i = 0; i < rover_quantity; i++) {
        udparray_[i] = new async_comm::UDP(local_host[i], local_port[i], remote_host[i], remote_port[i]);

        udp_ = udparray_[i];

        //if udp_ does not point to an object that has an init() method, throw an error.
        //Otherwise initialize udp
        if (!udp_->init())
        {
            throw std::runtime_error("Failed to initialize Base to Rover " + std::to_string(i+1) + " receive UDP");
        }

        rtcm_.registerCallback([this](uint8_t* buf, size_t size)
        {
            //std::cerr << "buf1 = " << buf << "\n";
            //std::cerr << "size1 = " << size << "\n";
            this->udp_->send_bytes(buf, size);
        });
    }

}

UBLOX::~UBLOX()
{
    if (udp_)
        delete udp_;

    if (log_file_.is_open())
        log_file_.close();
}

void UBLOX::udp_read_cb(const uint8_t* buf, size_t size)
{

    assert(type_ == ROVER);
    for (int i = 0; i < size; i++)
    {
        rtcm_.read_cb(buf[i]);
    }
}

void UBLOX::serial_read_cb(const uint8_t *buf, size_t size)
{

    for (int i = 0; i < size; i++)
    {
        /// TODO: don't give parsers data they don't need
        if (ubx_.parsing_message())
        {
            ubx_.read_cb(buf[i]);
        }
        else if (rtcm_.parsing_message() && type_ != NONE)
        {
            rtcm_.read_cb(buf[i]);
        }
        else
        {
            ubx_.read_cb(buf[i]);
            rtcm_.read_cb(buf[i]);
        }
    }

    if (log_file_.is_open())
    {
        log_file_.write((const char*)buf, size);
    }
}

void UBLOX::rtcm_complete_cb(const uint8_t *buf, size_t size)
{
    assert (type_ == ROVER || type_ == BASE);
    if (type_ == ROVER)
        serial_.send_bytes(buf, size);
    else if (type_ == BASE)
        udp_->send_bytes(buf, size);
}
}
