
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

v0.11	  Add 'b' to log file name "u_yy_mm_dd_hh_min_ss
11/26/19  If USB device inserted and memory space is available then 
	  copy log file(s) to USB drive on power down button push
	  If internal memory available is <100MB, delete oldest files until
	  available memory is >100MB
	  Minimum disk space cutoff reduced from 150MB to 50MB, this should not happen
	  now that we are managing the internal memory when 100MB of free space is reached
	  Power down button must be asserted for 1sec prior to powering down

v0.12	  Toggle buzzer (every second) if uSB memory full
12/03/19

v0.13	  ManageLogfiles() issue with file name error fixed (recycle bin folder)
06/24/20  Gracefully shutdown Linux OS

v0.14	  Do not enter usb powered mode
07/30/20 

v0.15	  Copy files if shutdown in startup up states(idle/powered)
09/05/20  Reset usb drive (GPS power) if device (/dev/sdx) not found
	  Find newest file 

v0.16	  Lockup if power button pushed while in SYSTEM_STATE_STARTUP
10/25/20  fixed