#ifndef UBLOX_H
#define UBLOX_H

#define UBLOX_BUFFER_SIZE 256

#include <stdint.h>

#include "async_comm/serial.h"

#include <iostream>

//int udp();
//void callback(uint8_t, size_t);

class UBLOX {
public:
    UBLOX(std::string port);
  enum {
    FIX_TYPE_NO_FIX = 0x00,
    FIX_TYPE_DEAD_RECKONING = 0x01,
    FIX_TYPE_2D = 0x02,
    FIX_TYPE_3D = 0x03,
    FIX_TYPE_GPS_AND_DEAD_RECKONING = 0x04,
    FIX_TYPE_TIME_ONLY = 0x05,
  };
  
  enum {
    START_BYTE_1 = 0xB5,
    START_BYTE_2 = 0x62,
  };
  
  enum {
    NMEA_START_BYTE1 = '$',
    NMEA_START_BYTE2 = 'G',
  };
  
  enum {
    CLASS_NAV = 0x01, //    Navigation Results Messages: Position, Speed, Time, Acceleration, Heading, DOP, SVs used
    CLASS_RXM = 0x02, //    Receiver Manager Messages: Satellite Status, RTC Status
    CLASS_INF = 0x04, //    Information Messages: Printf-Style Messages, with IDs such as Error, Warning, Notice
    CLASS_ACK = 0x05, //    Ack/Nak Messages: Acknowledge or Reject messages to CFG input messages
    CLASS_CFG = 0x06, //    Configuration Input Messages: Set Dynamic Model, Set DOP Mask, Set Baud Rate, etc.
    CLASS_UPD = 0x09, //    Firmware Update Messages: Memory/Flash erase/write, Reboot, Flash identification, etc.
    CLASS_MON = 0x0A, //    Monitoring Messages: Communication Status, CPU Load, Stack Usage, Task Status
    CLASS_AID = 0x0B, //    AssistNow Aiding Messages: Ephemeris, Almanac, other A-GPS data input
    CLASS_TIM = 0x0D, //    Timing Messages: Time Pulse Output, Time Mark Results
    CLASS_ESF = 0x10, //    External Sensor Fusion Messages: External Sensor Measurements and Status Information
    CLASS_MGA = 0x13, //    Multiple GNSS Assistance Messages: Assistance data for various GNSS
    CLASS_LOG = 0x21, //    Logging Messages: Log creation, deletion, info and retrieva
  };
  
  enum {
    ACK_ACK = 0x01, // Message Acknowledged
    ACK_NACK = 0x00, // Message Not-Acknowledged
  };
  
  enum {
    AID_ALM = 0x30, // Poll GPS Aiding Almanac Data
    AID_AOP = 0x33, // AssistNow Autonomous data
    AID_EPH = 0x31, // GPS Aiding Ephemeris Input/Output Message
    AID_HUI = 0x02, // Poll GPS Health, UTC, ionosphere parameters
    AID_INI = 0x01, // Aiding position, time, frequency, clock drift
  };

  enum {
    CFG_ANT = 0x13, 	// Get/Set Antenna Control Settings
    CFG_BATCH = 0x93, 	// Get/Set Get/Set data batching configuration
    CFG_CFG = 0x09, 	// Command Clear, Save and Load configurations
    CFG_DAT = 0x06, 	// Get The currently defined Datum
    CFG_DGNSS = 0x70, 	// Get/Set DGNSS configuration
    CFG_DOSC = 0x61, 	// Get/Set Disciplined oscillator configuration
    CFG_DYNSEED = 0x85, // Set Programming the dynamic seed for the host...
    CFG_ESRC = 0x60, 	// Get/Set External synchronization source configuration
    CFG_FIXSEED = 0x84, // Set Programming the fixed seed for host...
    CFG_GEOFENCE = 0x69,// Get/Set Geofencing configuration
    CFG_GNSS = 0x3E, 	// Get/Set GNSS system configuration
    CFG_HNR = 0x5C, 	// Get/Set High Navigation Rate Settings
    CFG_INF = 0x02, 	// Poll Request Poll configuration for one protocol
    CFG_ITFM = 0x39, 	// Get/Set Jamming/Interference Monitor configuration
    FG_LOGFILTER = 0x47,// Get/Set Data Logger Configuration
    CFG_MSG = 0x01, 	// Poll Request Poll a message configuration
    CFG_NAV5 = 0x24, 	// Get/Set Navigation Engine Settings
    CFG_NAVX5 = 0x23, 	// Get/Set Navigation Engine Expert Settings
    CFG_NMEA = 0x17, 	// Get/Set NMEA protocol configuration (deprecated)
    CFG_ODO = 0x1E, 	// Get/Set Odometer, Low_speed COG Engine Settings
    CFG_PM2 = 0x3B, 	// Get/Set Extended Power Management configuration
    CFG_PMS = 0x86, 	// Get/Set Power Mode Setup
    CFG_PRT = 0x00, 	// Poll Request Polls the configuration for one I/O Port
    CFG_PWR = 0x57, 	// Set Put receiver in a defined power state.
    CFG_RATE = 0x08, 	// Get/Set Navigation/Measurement Rate Settings
    CFG_RINV = 0x34, 	// Get/Set Contents of Remote Inventory
    CFG_RST = 0x04, 	// Command Reset Receiver / Clear Backup Data Structures
    CFG_RXM = 0x11, 	// Get/Set RXM configuration
    CFG_SBAS = 0x16, 	// Get/Set SBAS Configuration
    CFG_SMGR = 0x62, 	// Get/Set Synchronization manager configuration
    CFG_TMODE2 = 0x3D, 	// Get/Set Time Mode Settings 2
    CFG_TMODE3 = 0x71, 	// Get/Set Time Mode Settings 3
    CFG_TP5 = 0x31, 	// Poll Request Poll Time Pulse Parameters for Time Pulse 0
    CFG_TXSLOT = 0x53, 	// Set TX buffer time slots configuration
    CFG_USB = 0x1B, 	// Get/Set USB Configuration
    CFG_VALDEL = 0x8C, //Deletes values corresponding to...
    CFG_VALGET = 0x8B, //Get configuration items
    CFG_VALSET = 0x8A, //Set values correpsonding to provided...
  };
  
  enum {
    NAV_AOPSTATUS = 0x60,	// Periodic/Polled AssistNow Autonomous Status
    NAV_ATT = 0x05,		// Periodic/Polled Attitude Solution
    NAV_CLOCK = 0x22,		// Periodic/Polled Clock Solution
    NAV_DGPS = 0x31,		// Periodic/Polled DGPS Data Used for NAV
    NAV_DOP = 0x04,		// Periodic/Polled Dilution of precision
    NAV_EOE = 0x61,		// Periodic End Of Epoch
    NAV_GEOFENCE = 0x39,	// Periodic/Polled Geofencing status
    NAV_HPPOSECEF = 0x13,	// Periodic/Polled High Precision Position Solution in ECEF
    NAV_HPPOSLLH = 0x14,	// Periodic/Polled High Precision Geodetic Position Solution
    NAV_ODO = 0x09,		// Periodic/Polled Odometer Solution
    NAV_ORB = 0x34,		// Periodic/Polled GNSS Orbit Database Info
    NAV_POSECEF = 0x01,		// Periodic/Polled Position Solution in ECEF
    NAV_POSLLH = 0x02,		// Periodic/Polled Geodetic Position Solution
    NAV_PVT = 0x07,		// Periodic/Polled Navigation Position Velocity Time Solution
    NAV_RELPOSNED = 0x3C,	// Periodic/Polled Relative Positioning Information in NED frame
    NAV_RESETODO = 0x10,	// Command Reset odometer
    NAV_SAT = 0x35,		// Periodic/Polled Satellite Information
    NAV_SBAS = 0x32,		// Periodic/Polled SBAS Status Data
    NAV_SOL = 0x06,		// Periodic/Polled Navigation Solution Information
    NAV_STATUS = 0x03,		// Periodic/Polled Receiver Navigation Status
    NAV_SVINFO = 0x30,		// Periodic/Polled Space Vehicle Information
    NAV_SVIN = 0x3B,		// Periodic/Polled Survey-in data
    NAV_TIMEBDS = 0x24,		// Periodic/Polled BDS Time Solution
    NAV_TIMEGAL = 0x25,		// Periodic/Polled Galileo Time Solution
    NAV_TIMEGLO = 0x23,		// Periodic/Polled GLO Time Solution
    NAV_TIMEGPS = 0x20,		// Periodic/Polled GPS Time Solution
    NAV_TIMELS = 0x26,		// Periodic/Polled Leap second event information
    NAV_TIMEUTC = 0x21,		// Periodic/Polled UTC Time Solution
    NAV_VELECEF = 0x11,		// Periodic/Polled Velocity Solution in ECEF
    NAV_VELNED = 0x12,		// Periodic/Polled Velocity Solution in NED
  };
  
  typedef enum {
    START,    
    GOT_START_FRAME,
    GOT_CLASS,
    GOT_MSG_ID,
    GOT_LENGTH1,
    GOT_LENGTH2,
    GOT_PAYLOAD,
    GOT_CK_A,
    GOT_CK_B,
    DONE,
  } parse_state_t;
  
  typedef struct {
    uint8_t clsID;
    uint8_t msgID;
  }__attribute__((packed)) ACK_ACK_t;
  
  typedef struct {
    uint8_t clsID;
    uint8_t msgID;
  }__attribute__((packed)) ACK_NACK_t;
  
  typedef struct {
    uint8_t msgClass;
    uint8_t msgID;
    uint8_t rate;
  }__attribute__((packed)) CFG_MSG_t;
  
  typedef struct {
    enum {
      DYNMODE_PORTABLE = 0,
      DYNMODE_STATIONARY = 2,
      DYNMODE_PEDESTRIAN = 3,
      DYNMODE_AUTOMOTIVE = 4,
      DYNMODE_SEA = 5,
      DYNMODE_AIRBORNE_1G = 6,
      DYNMODE_AIRBORNE_2G = 7,
      DYNMODE_AIRBORNE_4G = 8
    };
    enum {
      FIXMODE_2D_ONLY = 1,
      FIXMODE_3D_ONLY = 2,
      FIXMODE_AUTO = 3,
    };

    enum{
      UTC_STANDARD_AUTO = 0, // receiver selects based on GNSS configuration (see GNSS time bases).
      UTC_STANDARD_USA = 3, // UTC as operated by the U.S. Naval Observatory (USNO); derived from GPS time
      UTC_STANDARD_RUS = 6, // UTC as operated by the former Soviet Union; derived from GLONASS time
      UTC_STANDARD_CHN = 7, // UTC as operated by the National Time Service Center, China; derived from BeiDou time
    };
    
    enum {
      MASK_DYN            = 0b00000000001, // Apply dynamic model settings
      MASK_MINEL 	  = 0b00000000010, // Apply minimum elevation settings
      MASK_POSFIXMODE     = 0b00000000100, // Apply fix mode settings
      MASK_DRLIM 	  = 0b00000001000, // Reserved
      MASK_POSMASK 	  = 0b00000010000, // Apply position mask settings
      MASK_TIMEMASK 	  = 0b00000100000, // Apply time mask settings
      MASK_STATICHOLDMASK = 0b00001000000, // Apply static hold settings
      MASK_DGPSMASK 	  = 0b00010000000, // Apply DGPS settings.
      MASK_CNOTHRESHOLD   = 0b00100000000, // Apply CNO threshold settings (cnoThresh, cnoThreshNumSVs).
      MASK_UTC 	          = 0b10000000000, // Apply UTC settings
    };
    
    uint16_t mask;
    uint8_t dynModel;
    uint8_t fixMode;  
    int32_t fixedAlt; // (1e-2 m) Fixed altitude (mean sea level) for 2D fix mode.
    uint32_t fixedAltVar; // (0.0001 m^2)Fixed altitude variance for 2D mode.
    int8_t minElev; // (deg) Minimum Elevation for a GNSS satellite to be used in NAV
    uint8_t drLimit; // s Reserved
    uint16_t pDop; // (0.1) - Position DOP Mask to use
    uint16_t tDop; // (0.1) - Time DOP Mask to use
    uint16_t pAcc; // m Position Accuracy Mask
    uint16_t tAcc; // m Time Accuracy Mask
    uint8_t staticHoldThr; // esh cm/s Static hold threshold
    uint8_t dgnssTimeout; // s DGNSS timeout
    uint8_t cnoThreshNumS; // Vs - Number of satellites required to have C/N0 above cnoThresh for a fix to be attempted
    uint8_t cnoThresh; // dBHz C/N0 threshold for deciding whether to attempt a fix
    uint8_t reserved1[2]; //  - Reserved
    uint16_t staticHoldMax; // Dist m Static hold distance threshold (before quitting static hold)
    uint8_t utcStandard; // - UTC standard to be used:
    uint8_t reserved2[5];
    
  }__attribute__((packed)) CFG_NAV5_t;
  
  typedef struct {
    enum {
      PORT_I2C = 0,
      PORT_UART1 = 1,
      PORT_USB = 3,
      PORT_SPI = 4
    };
    enum {
      CHARLEN_8BIT = 0b11000000,
      PARITY_NONE  = 0b100000000000,
      STOP_BITS_1  = 0x0000          
    };
    enum {
      IN_UBX   = 0b00000001,
      IN_NMEA  = 0b00000010,
      IN_RTCM  = 0b00000100,
      IN_RTCM3 = 0b00100000,
    };
    enum {
      OUT_UBX   = 0b00000001,
      OUT_NMEA  = 0b00000010,
      OUT_RTCM3 = 0b00100000,
    };

    uint8_t portID;
    uint8_t reserved1;
    uint16_t txReady;
    uint32_t mode; // What the mode the serial data is on (See charlen, parity and stop bits)
    uint32_t baudrate;
    uint16_t inProtoMask; // Which input protocols are enabled
    uint16_t outProtoMask; // Which output protocols are enabled
    uint16_t flags;
    uint8_t reserved2[2];
  }__attribute__((packed)) CFG_PRT_t;

    typedef struct {
    enum {
      VALGET_RAM = 0,
      VALGET_BBR = 1,
      VALGET_FLASH = 2,
      VALGET_DEFAULT = 7,
    };
    enum {
      VALGET_REQUEST = 0,
      VALGET_POLL  = 1,     
    };

    enum {
      VALGET_MSG_NAV_STATUS = 0x2091001d, //CFG-MSGOUT-UBX_NAV_STATUS_USB, tells the output rate of ubx-nav-status on usb
      VALGET_USB_ENABLED = 0x10650001,
      VALGET_I2C_ENABLED = 0x10510003,
      VALGET_DGNSSMODE = 0x20140011, //CFG-NAVHPG-DGNSSMODE Differential corrections mode
      VALGET_FIXMODE = 0x20110011,
      VALGET_INIFIX3D = 0x10110013,
      VALGET_WKNROLLOVER = 0x30110017,
    };

    enum { //incoming messages enabled
        VALGET_IN_UBX = 0x10770001, //CFG-USBINPROT-UBX
        VALGET_IN_NMEA = 0x10770002, //CFG-USBINPROT-NMEA
        VALGET_IN_RTCM3X = 0x10770004, //CFG-USBINPROT-RTCM3X
    };

    enum { //outgoing messages enabled
        VALGET_OUT_UBX = 0x10780001, //CFG-USBOUTPROT-UBX
        VALGET_OUT_NMEA = 0x10780002, //CFG-USBOUTPROT-NMEA
        VALGET_OUT_RTCM3X = 0x10780004, //CFG-USBOUTPROT-RTCM3X
    };

    enum { //outgoing message rates for RTCM 3x on usb type U1
        //suggested messages for stationary base
        RTCM_1005USB = 0x209102c0, //CFG-MSGOUT-RTCM_3X_TYPE1005_USB
        RTCM_1074USB = 0x20910361, //CFG-MSGOUT-RTCM_3X_TYPE1074_USB
        RTCM_1084USB = 0x20910366, //CFG-MSGOUT-RTCM_3X_TYPE1084_USB
        RTCM_1094USB = 0x2091036b, //CFG-MSGOUT-RTCM_3X_TYPE1094_USB
        RTCM_1124USB = 0x20910370, //CFG-MSGOUT-RTCM_3X_TYPE1124_USB
        RTCM_1230USB = 0x20910306, //CFG-MSGOUT-RTCM_3X_TYPE1230_USB
        //suggested messages for mobile base
        RTCM_4072_0USB = 0x20910301, //CFG-MSGOUT-RTCM_3X_TYPE4072_0_USB
        RTCM_4072_1USB = 0x20910384, //CFG-MSGOUT-RTCM_3X_TYPE4072_1_USB
        RTCM_1077USB = 0x209102cf, //CFG-MSGOUT-RTCM_3X_TYPE1077_USB
        RTCM_1087USB = 0x209102d4, //CFG-MSGOUT-RTCM_3X_TYPE1087_USB
        RTCM_1097USB = 0x2091031b, //CFG-MSGOUT-RTCM_3X_TYPE1097_USB
        RTCM_1127USB = 0x209102d9, //CFG-MSGOUT-RTCM_3X_TYPE1127_USB
        //!!!!also use RTCM_1230USB above!!!///
    };

    uint8_t version; //0 poll request, 1 poll (receiver to return config data key and value pairs)
    uint8_t layer;
    uint8_t reserved1[2];
    uint32_t cfgDataKey;
    uint64_t cfgData;
  }__attribute__((packed)) CFG_VALGET_t;

    typedef struct {
    enum {
      VALSET_RAM = 0b00000001,
      VALSET_BBR = 0b00000010,
      VALSET_FLASH = 0b00000100,
      VALSET_DEFAULT = 0b01000000,
    };

    enum {
      VALSET_0 = 0b00000000,
      VALSET_1  = 0b00000001,     
    };

    enum {
       VALSET_float = 2,
       VALSET_fixed  = 3,
    };

    enum {
      VALSET_MSG_NAV_STATUS = 0x2091001d, //CFG-MSGOUT-UBX_NAV_STATUS_USB, tells the output rate of ubx-nav-status on usb    
      VALSET_I2C_ENABLED_data = 0x10510003, //this is concatenated.  Should probably change to a more permanent format
      VALSET_DGNSSMODE = 0x20140011, //CFG-NAVHPG-DGNSSMODE Differential corrections mode
    };

    enum { //incoming messages enabled
        VALSET_IN_UBX = 0x10770001, //CFG-USBINPROT-UBX
        VALSET_IN_NMEA = 0x10770002, //CFG-USBINPROT-NMEA
        VALSET_IN_RTCM3X = 0x10770004, //CFG-USBINPROT-RTCM3X
    };

    enum { //outgoing messages enabled
        VALSET_OUT_UBX = 0x10780001, //CFG-USBOUTPROT-UBX
        VALSET_OUT_NMEA = 0x10780002, //CFG-USBOUTPROT-NMEA
        VALSET_OUT_RTCM3X = 0x10780004, //CFG-USBOUTPROT-RTCM3X
    };

    enum { //outgoing message rates for RTCM 3x on usb type U1
        //suggested messages for stationary base
        RTCM_1005USB = 0x209102c0, //CFG-MSGOUT-RTCM_3X_TYPE1005_USB
        RTCM_1074USB = 0x20910361, //CFG-MSGOUT-RTCM_3X_TYPE1074_USB
        RTCM_1084USB = 0x20910366, //CFG-MSGOUT-RTCM_3X_TYPE1084_USB
        RTCM_1094USB = 0x2091036b, //CFG-MSGOUT-RTCM_3X_TYPE1094_USB
        RTCM_1124USB = 0x20910370, //CFG-MSGOUT-RTCM_3X_TYPE1124_USB
        RTCM_1230USB = 0x20910306, //CFG-MSGOUT-RTCM_3X_TYPE1230_USB
        //suggested messages for mobile base
        RTCM_4072_0USB = 0x20910301, //CFG-MSGOUT-RTCM_3X_TYPE4072_0_USB
        RTCM_4072_1USB = 0x20910384, //CFG-MSGOUT-RTCM_3X_TYPE4072_1_USB
        RTCM_1077USB = 0x209102cf, //CFG-MSGOUT-RTCM_3X_TYPE1077_USB
        RTCM_1087USB = 0x209102d4, //CFG-MSGOUT-RTCM_3X_TYPE1087_USB
        RTCM_1097USB = 0x2091031b, //CFG-MSGOUT-RTCM_3X_TYPE1097_USB
        RTCM_1127USB = 0x209102d9, //CFG-MSGOUT-RTCM_3X_TYPE1127_USB
        //!!!!also use RTCM_1230USB above!!!///
    };
    enum {
        RATE_10 = 10,
    };

    uint8_t version; //0 poll request, 1 poll (receiver to return config data key and value pairs)
    uint8_t layer;
    uint8_t reserved1[2];
    uint32_t cfgDataKey;
    uint8_t cfgData;

  }__attribute__((packed)) CFG_VALSET_t;
  
  typedef struct {
    enum {
      TIME_REF_UTC = 0,
      TIME_REF_GPS = 1,
      TIME_REF_GLONASS = 2,
      TIME_REF_BUIDOU = 3,
      TIME_REF_GALILEO = 4
    };
    uint16_t measRate; // (ms) The elapsed time between GNSS measurements which defines the rate
    uint16_t navRate; // (cycles) The ratio between the number of measurements and the number of navigation solutions, e.g. 5 means five measurements for every navigation solution
    uint16_t timeRef; // Time system to which measurements are aligned
  }__attribute__((packed)) CFG_RATE_t;
  
  typedef struct  {
    enum {
      VALIDITY_FLAGS_VALIDDATE= 0b01, // Valid UTC Date (see Time Validity section for details)
      VALIDITY_FLAGS_VALIDTIME = 0b10, // Valid UTC Time of Day (see Time Validity section for details)
      VALIDITY_FLAGS_FULLYRESOLVED = 0b100, // UTC Time of Day has been fully resolved (no seconds uncertainty)
    };
    
    enum {
      FIX_STATUS_GNSS_FIX_OK            = 0b00000001, // Valid Fix
      FIX_STATUS_DIFF_SOLN              = 0b00000010, // Differential Corrections were applied
      FIX_STATUS_PSM_STATE_NOT_ACTIVE   = 0b00000000, 
      FIX_STATUS_PSM_STATE_ENABLED      = 0b00000100, 
      FIX_STATUS_PSM_STATE_ACQUISITION  = 0b00001000, 
      FIX_STATUS_PSM_STATE_TRACKING     = 0b00001100, 
      FIX_STATUS_PSM_STATE_POWER_OPTIMIZED_TRACKING   = 0b00010000,
      FIX_STATUS_PSM_STATE_INACTIVE     = 0b00010100,
      FIX_STATUS_HEADING_VALID          = 0b00100000,
      FIX_STATUS_CARR_SOLN_NONE         = 0b00000000,
      FIX_STATUS_CARR_SOLN_FLOAT        = 0b01000000,
      FIX_STATUS_CARR_SOLN_FIXED        = 0b10000000,
    };
    
    uint32_t iTOW; // ms GPS time of week of the  navigation epoch . See the  description of iTOW for details.
    uint16_t year; // y Year (UTC)
    uint8_t month; // month Month, range 1..12 (UTC)
    uint8_t day; // d Day of month, range 1..31 (UTC)
    uint8_t hour; // h Hour of day, range 0..23 (UTC)
    uint8_t min; // min Minute of hour, range 0..59 (UTC)
    uint8_t sec; // s Seconds of minute, range 0..60 (UTC)
    uint8_t valid; // - Validity flags (see  graphic below )
    uint32_t tAcc; // ns Time accuracy estimate (UTC)
    int32_t nano; // ns Fraction of second, range -1e9 .. 1e9 (UTC)
    uint8_t fixType; // - GNSSfix Type:
    uint8_t flags; // - Fix status flags (see  graphic below )
    uint8_t flags2; // - Additional flags (see  graphic below )
    uint8_t numSV; // - Number of satellites used in Nav Solution
    int32_t lon; // 1e-7 deg Longitude
    int32_t lat; // 1e-7 deg Latitude
    int32_t height; // mm Height above ellipsoid
    int32_t hMSL; // mm Height above mean sea level
    uint32_t hAcc; // mm Horizontal accuracy estimate
    uint32_t vAcc; // mm Vertical accuracy estimate
    int32_t velN; // mm/s NED north velocity
    int32_t velE; // mm/s NED east velocity
    int32_t velD; // mm/s NED down velocity
    int32_t gSpeed; // mm/s Ground Speed (2-D)
    int32_t headMot; // 1e-5 deg Heading of motion (2-D)
    uint32_t sAcc; // mm/s Speed accuracy estimate
    uint32_t headAcc; // 1e-5 deg Heading accuracy estimate (both motion and vehicle)
    uint16_t pDOP; // 0.01  - Position DOP
    uint8_t reserved1; //[6] - Reserved
    int32_t headVeh; // 1e-5 deg Heading of vehicle (2-D)
    int16_t magDec; // 1e-2 deg Magnetic declination
    uint16_t magAcc; // 1e-2 deg Magnetic declination accuracy
  }__attribute__((packed)) NAV_PVT_t;

  typedef struct
  {
    uint32_t iTOW; // ms GPS time of week of the  navigation epoch . See the  description of iTOW for details.
    int32_t ecefX; // cm ECEF X coordinate
    int32_t ecefY; // cm ECEF Y coordinate
    int32_t ecefZ; // cm ECEF Z coordinate
    uint32_t pAcc; // cm Position Accuracy Estimate
  }__attribute__((packed)) NAV_POSECEF_t;

  typedef struct
  {
    uint32_t iTOW; // ms GPS time of week of the  navigation epoch . See the  description of iTOW for details.
    int32_t ecefVX; // cm ECEF X velocity
    int32_t ecefVY; // cm ECEF Y velocity
    int32_t ecefVZ; // cm ECEF Z velocity
    uint32_t sAcc; // cm Speed Accuracy Estimate
  }__attribute__((packed)) NAV_VELECEF_t;
  
  typedef union {
    uint8_t buffer[UBLOX_BUFFER_SIZE];
    ACK_ACK_t ACK_ACK;
    ACK_NACK_t ACK_NACK;
    CFG_MSG_t CFG_MSG;
    CFG_PRT_t CFG_PRT;
    CFG_RATE_t CFG_RATE;
    CFG_NAV5_t CFG_NAV5;
    CFG_VALSET_t CFG_VALSET;
    CFG_VALGET_t CFG_VALGET;
    NAV_PVT_t NAV_PVT;
    NAV_POSECEF_t NAV_POSECEF;
    NAV_VELECEF_t NAV_VELECEF;
  } UBX_message_t;
  
  void init(int rover);
  void config_rover();
  void config_base();
  
  void read(double* lla, float* vel, uint8_t &fix_type, uint32_t& t_ms);
  void read_cb(uint8_t byte);
  inline volatile bool new_data() { return new_data_; }
  uint32_t volatile num_messages_received() { return num_messages_received_; }

  void lla(double* lla) const;
  void ned(float* vel) const;
  uint8_t fix_type() const;
  void posECEF(double* pos) const;
  void velECEF(double* vel) const;

  
private:
  bool detect_baudrate();
  void convert_data();
  void enable_message(uint8_t msg_cls, uint8_t msg_id, uint8_t rate);
  void set_baudrate(const uint32_t baudrate);
  void set_dynamic_mode();
  void set_nav_rate(uint8_t period_ms);
  bool decode_message();
  void calculate_checksum(const uint8_t msg_cls, const uint8_t msg_id, const uint16_t len, const UBX_message_t payload, uint8_t &ck_a, uint8_t &ck_b) const;
  bool send_message(uint8_t msg_class, uint8_t msg_id, UBX_message_t& message, uint16_t len);
  
  uint32_t current_baudrate_ = 115200;
  const uint32_t baudrates[5] = {115200, 19200, 57600, 9600, 38400};
  
  UBX_message_t out_message_;
  UBX_message_t in_message_;
    
  uint16_t buffer_head_ = 0;
  bool got_message_ = false;
  bool got_ack_ = false;
  bool got_nack_ = false;
  parse_state_t parse_state_;
  uint8_t message_class_;
  uint8_t message_type_;
  uint16_t length_;  
  uint8_t ck_a_;
  uint8_t ck_b_;
  uint32_t num_errors_ = 0;
  uint32_t num_messages_received_ = 0;
  int message_sent = 0;
  
  double lla_[3];
  float vel_[3];
  
  bool looking_for_nmea_;
  uint8_t prev_byte_ = 0;
  
  async_comm::Serial serial_;
  
  volatile bool new_data_;
  NAV_PVT_t nav_message_;
  NAV_POSECEF_t pos_ecef_;
  NAV_VELECEF_t vel_ecef_;
  CFG_VALGET_t cfg_get_message_;
};

#endif // UBLOX_H

