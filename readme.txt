
/*|***************************************************************************/
/*|PROJECT:  Edison Base Station
/*|***************************************************************************/

version
mm/dd/yy

v0.00     Initial release
06/25/18

v0.01     Initial release
06/25/18

v0.02     Enable low battery pin internal pull up
07/16/18  Fix up Swiftnav GPS get time as per Tagr firmware

v0.03     Invert GSP signal (back to original state)
09/22/18  

v0.04     Change green led cadence 
10/28/18  Baudrate 460800bps

v0.05     Remove check for GPS message (0x102)
11/02/18  

v0.06     Take LEDs off of PWM peripheral and use 
11/07/18  timers for led blink 
          (PWM period cannot exceed 220msec on Edison)
 	  Fix shutdown buzzer not stopping after 8 chirps

v0.07     Low battery power down re-enabled
11/08/18  

v0.08     Auto shutdown was originally correct in v0.06, went back to that 
11/08/18  shutdown condition.  
	  GPS power up delay increased from 5 seconds to 30 seconds
	  change 'DONE' message in log file to 'Shut Down'

v0.09     Turn Leds off when in USB powered (charging) state
11/09/18  
