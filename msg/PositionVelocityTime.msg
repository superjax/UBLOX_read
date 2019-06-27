Header header

uint16 year  	# y Year (UTC)
uint8 month  	# month Month, range 1..12 (UTC)
uint8 day  	# d Day of month, range 1..31 (UTC)
uint8 hour  	# h Hour of day, range 0..23 (UTC)
uint8 min  	# min Minute of hour, range 0..59 (UTC)
uint8 sec  	# s Seconds of minute, range 0..60 (UTC)
int32 nano  	# ns Fraction of second, range -1e9 .. 1e9 (UTC)
uint32 tAcc  	# ns Time accuracy estimate (UTC)

uint8 valid  		# - Validity flags (see below )
uint8 fixType 	# - GNSSfix Type:
uint8 flags  		# - Fix status flags (see  graphic below )
uint8 flags2  	# - Additional flags (see  graphic below )
uint8 numSV  		# - Number of satellites used in Nav Solution
float64[3] lla 		# lat, lon, altitude (deg, deg, m)
float64 hMSL  		# Height above mean sea level (m)
float64 hAcc  		# Horizontal accuracy estimate (m)
float64 vAcc  		# Vertical accuracy estimate (m)
float64[3] velNED	# NED velocity (m/s)
float64 gSpeed  	# m/s Ground Speed (2-D)
float64 headMot 	# deg Heading of motion (2-D)
float64 sAcc  		# m/s Speed accuracy estimate
float64 headAcc		# deg Heading accuracy estimate (both motion and vehicle)
float64 pDOP  		# Position DOP
float64 headVeh  	# deg Heading of vehicle (2-D)

uint8 FIX_TYPE_NO_FIX = 0
uint8 FIX_TYPE_DEAD_RECKONING = 1
uint8 FIX_TYPE_2D = 2 
uint8 FIX_TYPE_3D = 3 
uint8 FIX_TYPE_GPS_AND_DEAD_RECKONING = 4 
uint8 FIX_TYPE_TIME_ONLY = 5 

uint8 FIX_STATUS_GNSS_FIX_OK            = 1 # Valid Fix
uint8 FIX_STATUS_DIFF_SOLN              = 2 # Differential Corrections were applied
uint8 FIX_STATUS_PSM_STATE_NOT_ACTIVE   = 0
uint8 FIX_STATUS_PSM_STATE_ENABLED      = 4
uint8 FIX_STATUS_PSM_STATE_ACQUISITION  = 8
uint8 FIX_STATUS_PSM_STATE_TRACKING     = 12
uint8 FIX_STATUS_PSM_STATE_POWER_OPTIMIZED_TRACKING   = 16
uint8 FIX_STATUS_PSM_STATE_INACTIVE     = 20
uint8 FIX_STATUS_HEADING_VALID          = 32
uint8 FIX_STATUS_CARR_SOLN_NONE         = 0
uint8 FIX_STATUS_CARR_SOLN_FLOAT        = 64
uint8 FIX_STATUS_CARR_SOLN_FIXED        = 128

uint8 VALIDITY_FLAGS_VALIDDATE= 1 			# Valid UTC Date (see Time Validity section for details)
uint8 VALIDITY_FLAGS_VALIDTIME = 2 		# Valid UTC Time of Day (see Time Validity section for details)
uint8 VALIDITY_FLAGS_FULLYRESOLVED = 4 	# UTC Time of Day has been fully resolved (no seconds uncertainty)
