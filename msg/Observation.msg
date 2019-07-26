Header header

time time           # time of observation (UTC Time inc. Leap Seconds)
uint8 sat 		    # satellite number (within constellation)
uint8 gnssID	    # gnssID

uint8 GNSSID_GPS = 0
uint8 GNSSID_SBAS = 1
uint8 GNSSID_GALILEO = 2
uint8 GNSSID_BEIDOU = 3
uint8 GNSSID_QZSS = 5
uint8 GNSSID_GLONASS = 6

float64 freq        # carrier-phase frequency (GLONASS uses different frequency for each satellite)
uint32 locktime		# carrier-phase lock counter (max=64500ms)
float64 cno			# Carrier-to-noise density ratio (signal strength)[dB-Hz]

float64 L      		# carrier-phase (cycle)
float64 P      		# pseudorange (m)
float64 D      		# doppler frequency (Hz)
float64 stdevL  	# carrier-phase stddev (cycle)
float64 stdevP  	# psuedorange stddev (m)
float64 stdevD  	# doppler stddev (Hz)

uint8 LLI 			# Loss-of-Lock Indicator
uint8 LLI_LOST = 1
uint8 LLI_HALF_CYCLE_AMB = 2

uint8 code 			# code indicator
uint8 CODE_L1C = 1
uint8 BEIDOU_CODE_L1I = 2
