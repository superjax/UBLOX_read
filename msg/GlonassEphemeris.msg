Header header

uint8 sat 					  # Satellite ID (within Constellation)
uint8 gnssID				  # Satellite ID (within Constellation)

uint8 GNSSID_GPS = 0
uint8 GNSSID_SBAS = 1
uint8 GNSSID_GALILEO = 2
uint8 GNSSID_BEIDOU = 3
uint8 GNSSID_QZSS = 5
uint8 GNSSID_GLONASS = 6

time toe        	# epoch of epherides (UTC inc. leap-seconds)
time tof            # message frame time (UTC inc. leap-seconds)

int32 iode          # IODE (0-6 bit of tb field)
int32 frq           # satellite frequency number
int32 svh			# satellite health
int32 sva			# satellite accuracy (ura)
int32 age   		# age of operation
float64[3] pos      # satellite position (ecef) (m)
float64[3] vel      # satellite velocity (ecef) (m/s)
float64[3] acc      # satellite acceleration (ecef) (m/s^2)
float64 taun		# SV clock bias (s)
float64 gamn   	  	# relative freq bias
float64 dtaun       # delay between L1 and L2 (s)