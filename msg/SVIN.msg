Header header

uint32 dur 			# Passed survey-in observation time s
float64[3] meanXYZ			# Current survey-in mean position ECEF X coordinate cm
uint32 meanAcc		# Current survey-in mean position accuracy mm
uint32 obs 			# number of position observations used during survey-in
float64[3] meanXYZHP
uint8 valid 			# Survey-in postion validity flag, 1=valid, otherwise 0
uint8 active			# survey-in in progress flag, 1 = in-progress, otherwise 0
