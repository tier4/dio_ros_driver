uint8 NO_ERROR=0
uint8 CHIP_UNOPEN=1
uint8 GET_LINE_FAILED=2
uint8 SET_MODE_FAILED=3
uint8 GET_VALUE_FAILED=4
uint8 SET_VALUE_FAILED=5

Header header
uint16 status    # status has the following two fields
              	 # status[7:0] means failure port
	      	 # status[15:8] means Error code
	         # any failure is found on any specific port,
		 # the corresponding bit would be 1.
		 # if gpiochip cannot be opened, all bits would be 1.
uint16 raw_value # send raw value as is.
