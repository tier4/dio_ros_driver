uint8 NO_DIO_ERROR=0
uint8 NOT_OPEN_DIOCHIP=1
uint8 NOT_GET_LINE=2
uint8 NOT_SET_LINE_DIRECTION=3
uint8 NOT_GET_LINE_VALUE=4
uint8 NOT_SET_LINE_VALUE=5

Header header
uint16 status    # status has the following two fields
              	 # status[7:0] means failure port
      	 	 # status[15:8] means Error code
         	 # any failure is found on any specific port,
 		 # the corresponding bit would be 1.
 		 # if gpiochip cannot be opened, all bits would be 1.

uint16 user_value # value sent to user as topic
