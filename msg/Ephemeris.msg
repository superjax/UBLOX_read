Header header

uint8 sat 					  # Satellite ID (within Constellation)
uint8 gnssID				  # Satellite ID (within Constellation)

uint8 GNSSID_GPS = 0
uint8 GNSSID_SBAS = 1
uint8 GNSSID_GALILEO = 2
uint8 GNSSID_BEIDOU = 3
uint8 GNSSID_QZSS = 5
uint8 GNSSID_GLONASS = 6

time  toe                     # reference time ephemeris (UTC Time inc. leap seconds)                         [s]
time  toc                     # reference time (clock)   (UTC Time inc. leap seconds)                         [s]

uint32 tow                    # time of week in subframe1 the time of the leading bit edge of subframe 2      [s]
uint16 iodc                   # 10 bit issue of data (clock) 8 LSB bits will match the iode                   []
uint8  iode                   # 8 bit  issue of data (ephemeris)                                              []
uint16 week                   # 10 bit gps week 0-1023 (user must account for week rollover )                 [week]
uint32 toes                   # Time of ephemeris (seconds part)
uint32 tocs                   # Time of clock (seconds part)
uint8  health                 # 6 bit health parameter 0 if healthy unhealth othersize                        [0=healthy]
uint8  alert_flag             # 1 = URA may be worse than indicated                                           [0,1]
uint8  anti_spoof             # anti-spoof flag from 0=off 1=on                                               [0,1]
uint8  code_on_L2             # 0=reserved 1=P code on L2 2=C/A on L2                                         [0,1,2]
uint8  ura                    # User Range Accuracy lookup code 0 is excellent 15 is use at own risk          [0-15] see p. 83 GPSICD200C
uint8  L2_P_data_flag         # flag indicating if P is on L2 1=true                                          [0,1]
uint8  fit_interval_flag      # fit interval flag (four hour interval or longer) 0=4 fours 1=greater          [0,1]
uint16 age_of_data_offset     # age of data offset                                                            [s]
float64[4]   tgd              # group delay                                                                   [s]
float64   af2                 # polynomial clock correction coefficient (rate of clock drift)                 [s/s^2]
float64   af1                 # polynomial clock correction coefficient (clock drift)                         [s/s]
float64   af0                 # polynomial clock correction coefficient (clock bias)                          [s]
float64   m0                  # mean anomaly at reference time                                                [rad]
float64   delta_n             # mean motion difference from computed value                                    [rad/s]
float64   ecc                 # eccentricity                                                                  []
float64   sqrta               # square root of the semi-major axis                                            [m^(1/2)]
float64   omega0              # longitude of ascending node of orbit plane at weekly epoch                    [rad]
float64   i0                  # inclination angle at reference time                                           [rad]
float64   w                   # argument of perigee                                                           [rad]
float64   omegadot            # rate of right ascension                                                       [rad/s]
float64   idot                # rate of inclination angle                                                     [rad/s]
float64   cuc                 # amplitude of the cosine harmonic correction term to the argument of latitude  [rad]
float64   cus                 # amplitude of the sine harmonic correction term to the argument of latitude    [rad]
float64   crc                 # amplitude of the cosine harmonic correction term to the orbit radius          [m]
float64   crs                 # amplitude of the sine harmonic correction term to the orbit radius            [m]
float64   cic                 # amplitude of the cosine harmonic correction term to the angle of inclination  [rad]
float64   cis                 # amplitude of the sine harmonic correction term to the angle of inclination    [rad]


