Header header

uint16 refStationId 	# Reference Station ID. Must be in the range 0..4095
float64[3] relPosNED	# NED component of relative position vector (m)
float64 relPosLength 	# Length of relative position vector (m)
float64 relPosHeading 	# Heading of the relative position vector. (rad)
float64[3] relPosHPNED 	# High precision NED, the measurment portion less than a mm. (m)
float64 relPosHPLength 	# High precision Length, the measurment portion less than a mm. (m)
float64[3] accNED 		# Accuracy of relative position North component (m)
float64 accLength 		# Accuracy of Length of the relative position vector (m)
uint32 accHeading 		# Accuracy of heading of the relative position vector (rad)
uint32 flags 			# See enums below

uint32 FLAGS_GNSS_FIX_OK =            1
uint32 FLAGS_DIFF_SOLN =              2
uint32 FLAGS_REL_POS_VALID =          4
uint32 FLAGS_CARR_SOLN_FLOAT =        8
uint32 FLAGS_CARR_SOLN_FIXED =        16
uint32 FLAGS_IS_MOVING =              32
uint32 FLAGS_REF_POS_MISS =           64
uint32 FLAGS_REF_OBS_MISS =           128
uint32 FLAGS_REL_POS_HEADING_VALID =  256