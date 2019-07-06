Header header

uint32_t dur 			# Passed survey-in observation time s
uint32_t meanx			# Current survey-in mean position ECEF X coordinate cm
uint32_t meany 			# Current survey-in mean position ECEF Y coordinate cm
uint32_t meanz			# Current survey-in mean position ECEF Z coordinate cm
uint32_t meanAcc		# Current survey-in mean position accuracy mm
uint32_t obs 			# number of position observations used during survey-in
uint8_t valid 			# Survey-in postion validity flag, 1=valid, otherwise 0
uint8_t active			# survey-in in progress flag, 1 = in-progress, otherwise 0
