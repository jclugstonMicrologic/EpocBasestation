#include <mraa.h>
#include <time.h>
#include <string.h>
#include <sys/statvfs.h>

#define BASE_STATION_VERSION "BASE STATION v0.10 11/15/2018\n"   

#define SWIFTNAV
#ifdef SWIFTNAV
 #define RAW_GPS_BAUD_RATE  230400 //230400, 115200 for RTK version
#else
 #define RAW_GPS_BAUD_RATE  460800 //921600 
#endif

#define POWER_UP_TIME			30
#define LOSS_COMM_TIMEOUT		5
#define PRE_FLIGHT_TIMEOUT		1 //600

#define GNNS_MSGS_COMPLETE		2  

#define MIN_DISK_SPACE			150000000 /* (150MB of 800MB) */

/* define these for normal operation */
#define USE_OTG_MSD
#define TURN_OFF_WIFI /* is this necesseray for the base station??? */

/* test defines */
//#define AUTO_START
//#define DISABLE_PV_DETECT
//#define SIM_GPS_DATE_TIME
//#define SIM_RAW_DATA
//#define SKIP_SELF_TEST
//#define FILE_WRITE_TEST
//#define LED_TEST_ENABLED
//#define INIT_GNSS_ENABLED
//#define GPS_DISABLED

/** Local Constants and Types *************************************************/
#if 0
enum
{
	LED_BLINK =0,
	LED_SOLID 
};
#else
enum
{
	LED_OFF =0,
	LED_ON,
	LED_BLINK,
	LED_BLINK2
};
#endif

typedef enum
{
	SYSTEM_STATE_INIT =0,	
	SYSTEM_STATE_STARTUP,
	SYSTEM_STATE_IDLE,	
	SYSTEM_STATE_POWERED,
	SYSTEM_STATE_WAIT_GPS_TIME,
	SYSTEM_STATE_SELF_TEST,
//	SYSTEM_STATE_SELF_TEST_SETTLE,
	SYSTEM_STATE_SELF_TEST_FAIL,
	SYSTEM_STATE_SELF_TEST_PASS,
	SYSTEM_STATE_READY,
	SYSTEM_STATE_READY_GPS_LOST,
	SYSTEM_STATE_POWER_DOWN,
	SYSTEM_STATE_SHUT_DOWN,
	SYSTEM_STATE_USB_POWERED,
	
	SYSTEM_STATE_LED_TEST_STATE1,
	SYSTEM_STATE_LED_TEST_STATE2,
	SYSTEM_STATE_LED_TEST_STATE3,
	SYSTEM_STATE_LED_TEST_STATE4,	
	
	SYSTEM_STATE_LAST
	
}SystemStateTypeEnum;

typedef struct
{
	unsigned int stateTimer;
	SystemStateTypeEnum machState;
}MAIN_SYSTEM;

typedef struct
{
	int year;
	int month;
	int day;
	int hour;
	int min;
	int sec;
}GPS_DATE_TIME;

MAIN_SYSTEM MainSystem;
GPS_DATE_TIME GpsDateTime;

/** Local Variable Declarations ***********************************************/
FILE* fp;
char FileExt[] ="";

typedef struct
{
	int week;
	int time;	
}GPS_TOW;

typedef struct
{
	int16_t x;
	int16_t y;
	int16_t z;
}IMU_DATA;

typedef struct
{
	float heading;
	float pitch;
	float roll;
}SPATIAL_DATA;

GPS_TOW GpsTow;

char filename1[255];
char ImuFilename[255];


/*
*|----------------------------------------------------------------------------
*|  Routine: CheckFileExists
*|  Description:
*|  Retval:
*|----------------------------------------------------------------------------
*/
int CheckFileExists(char *filename)
{
	FILE *pFile;	
	
	pFile =fopen(filename,"r");
	
    if( pFile !=0 )
	{		
		fclose(pFile);			
		return 1;
	}	
	
	else
		return 0;
}

/*
*|----------------------------------------------------------------------------
*|  Routine: CreateLogFile
*|  Description:
*|  Retval:
*|----------------------------------------------------------------------------
*/
int CreateLogFile(char *pFileExt)
{	
	time_t t = time(NULL);		
	struct tm *tm = localtime(&t);	
	int retval =0;

#ifdef USE_OTG_MSD	
	sprintf(filename1, "/home/root/msd/%d_%d_%d_%d_%d_%d.%s",GpsDateTime.year,GpsDateTime.month,GpsDateTime.day,GpsDateTime.hour,GpsDateTime.min,GpsDateTime.sec,pFileExt);	
	//sprintf(filename1, "/home/root/msd/%d_%d_%d_%d_%d_%d.log",GpsDateTime.year,GpsDateTime.month,GpsDateTime.day,GpsDateTime.hour,GpsDateTime.min,GpsDateTime.sec);	
#else
	// use the SD card
	sprintf(filename1, "/media/sdcard/%d_%d_%d_%d_%d_%d.log",GpsDateTime.year,GpsDateTime.month,GpsDateTime.day,GpsDateTime.hour,GpsDateTime.min,GpsDateTime.sec);	
#endif

	printf("%s\n", filename1);
	
#ifdef USE_OTG_MSD	
	system("mkdir /home/root/msd");

	retval = system("mount -o offset=8192 /dev/mmcblk0p9 /home/root/msd");
	printf("%d\n", retval);
#else
	retval = system("mount /dev/mmcblk1p1 /media/sdcard");
	printf("%d\n", retval);
#endif	
			
#if 0			
	if( CheckFileExists(filename1) )
	{
		printf("\nFile exists, rename\n\n");		
		
		// rename to year +1, will make it obvious this has happened
		sprintf(filename1, "/home/root/msd/%d_%d_%d_%d_%d_%d.log",(tm->tm_year+1900),(tm->tm_mon+1),tm->tm_mday,tm->tm_hour,tm->tm_min,tm->tm_sec);	
	    //sprintf(filename1, "/home/root/msd/%d_%d_%d_%d_%d_%d.log",(GpsDateTime.year+1),GpsDateTime.month,GpsDateTime.day,GpsDateTime.hour,GpsDateTime.min,GpsDateTime.sec);			
	}
#endif
	
	fp = fopen(filename1,"w");
	
	if( fp ==0 )
	{
		printf("File create failed");
		
		return 0;
	}
	else
	{	
		retval =fprintf(fp, BASE_STATION_VERSION);
	
		fclose(fp);	
		
		if( retval >0 )
		{
			printf("File Init\n");
			return 1;
		}
		else
		{			
			printf("Write Fail\n");
			return 0;
		}
	}		
}


/*
*|----------------------------------------------------------------------------
*|  Routine: SimGpsTime
*|  Description:
*|  Retval:
*|----------------------------------------------------------------------------
*/
void SimGpsTime()
{
	GpsTow.week =1949;
	GpsTow.time =321548;

	GpsDateTime.year =2018;
	GpsDateTime.month =1;
	GpsDateTime.day  =1;
	GpsDateTime.hour =1;
	GpsDateTime.min =1;
	GpsDateTime.sec =1;	
}

/*
*|----------------------------------------------------------------------------
*|  Routine: ParseGpsTime
*|  Description:
*|  Retval:
*|----------------------------------------------------------------------------
*/
int ParseGpsTime(char *pRxBuf)
{
	// TIMEA,COM1,0,56.5,FINESTEERING,1948,43207.000,00000000,9924,14039;VALID,1.744794222e-09,1.285034918e-09,-18.00000000000,2017,5,7,11,59,49000,VALID*8e686cbf	
	// 5 commas, GpsWeek
	// 6 commas, GpsTime
	
	int index;	
	int commaCnt =0;	

	#ifdef SWIFTNAV	
		strcpy( FileExt, "sbp");
		return 1;
	#elif COMNAV
		strcpy( FileExt, "cnb");
	#else
		strcpy( FileExt, "log");
	#endif
	
	#if 1
	if(strstr(pRxBuf, "INVALID*") !=0 ||
	   strstr(pRxBuf, "ERROR") !=0
	  )
	{
		printf("GPS Time INVALID!!!\n");
		return 0;
	}
	#endif
	

	for( index=0; index<255; index++)
	{
		if( *(pRxBuf++) == ',' )
		{
			switch( commaCnt )	
			{
				case 4:
					//GpsWeek =1948;
					//memcpy( &GpsTow.week, pRxBuf, sizeof(GpsTow.week) );
					GpsTow.week =atoi(pRxBuf);
					printf("\nGPS Week %d\n", GpsTow.week);
					break;
				case 5:
					//GpsTime =390600;
					//memcpy( &GpsTow.time, pRxBuf, sizeof(GpsTow.time) );
					GpsTow.time =atoi(pRxBuf);
					printf("\nGPS Time %d\n", GpsTow.time);
					break;					
				case 12:					
					GpsDateTime.year =atoi(pRxBuf);
					break;							
				case 13:					
					GpsDateTime.month =atoi(pRxBuf);
					break;							
				case 14:					
					GpsDateTime.day =atoi(pRxBuf);
					break;							
				case 15:					
					GpsDateTime.hour =atoi(pRxBuf);
					break;							
				case 16:					
					GpsDateTime.min =atoi(pRxBuf);
					break;							
				case 17:					
					GpsDateTime.sec =atoi(pRxBuf);
					GpsDateTime.sec /=1000;
					break;												
			}
			
			commaCnt ++;
			
			if( commaCnt >17 )
			{				
				printf("\nGPS Time %d,%d,%d %d:%d:%d\n", GpsDateTime.year,GpsDateTime.month,GpsDateTime.day,GpsDateTime.hour,GpsDateTime.min,GpsDateTime.sec);
				
				if( GpsDateTime.year ==0 || GpsDateTime.month ==0 || GpsDateTime.day ==0 )
				{
					return 0;
				}
				else
					return 1;
			}
		}
	}
	
	return 0;
}


/*
*|----------------------------------------------------------------------------
*|  Routine: ParseSwiftNavTime
*|  Description:
*|  Retval:
*|----------------------------------------------------------------------------
*/
uint8_t ParseSwiftNavTime(char *pRxBuf)
{
	int k;
	uint8_t swiftNavTime =0;
	
	for(k=2; k<4096; k++)
	{
		//printf("%x", pRxBuf[k]);
		if( pRxBuf[k] ==0x01 && pRxBuf[k-1] ==0x02 && pRxBuf[k-2] ==0x55 )
		{
			//printf("SwiftNav GPS TIME %x %x %x %x %x %x %x %x\n", pRxBuf[k+1], pRxBuf[k+2], pRxBuf[k+4], pRxBuf[k+5], pRxBuf[k+6], pRxBuf[k+7],pRxBuf[k+8], pRxBuf[k+9]);
										
			GpsTow.week =(uint8_t)(pRxBuf[k+5]&0x000000ff)<<8 | (uint8_t)pRxBuf[k+4]&0x000000ff;
			GpsTow.time =(uint32_t)((uint8_t)(pRxBuf[k+9]&0x000000ff)<<24 | (uint8_t)(pRxBuf[k+8]&0x000000ff)<<16 | (uint8_t)(pRxBuf[k+7]&0x000000ff)<<8 | (uint8_t)(pRxBuf[k+6]));
						
			GpsTow.time /=1000;
						
			if( GpsTow.week >1990 && GpsTow.week <3000 )
			{
				swiftNavTime |=0x01;						
			}
						
			GpsDateTime.year =GpsTow.week;
			GpsDateTime.month =GpsTow.time;
						
			//GpsTow.time =gpsTime;
			//GpsTow.time /=1000;
			printf("GPS TIME %d %d\n", GpsTow.week, GpsTow.time);						
		//	break;
		}	
		if( pRxBuf[k] ==0x01 && pRxBuf[k-1] ==0x03 && pRxBuf[k-2] ==0x55)
		{
			//printf("SwiftNav UTC TIME %x %x %x %x %x %x %x %x\n", pRxBuf[k+1], pRxBuf[k+2], pRxBuf[k+4], pRxBuf[k+5], pRxBuf[k+6], pRxBuf[k+7],pRxBuf[k+8], pRxBuf[k+9]);						
						
			GpsTow.time =(uint8_t)(pRxBuf[k+8]&0x00ff)<<24 | (uint8_t)(pRxBuf[k+7]&0x00ff)<<16 | (uint8_t)(pRxBuf[k+6]&0x00ff)<<8 | (uint8_t)pRxBuf[k+5]&0x00ff;
			GpsTow.time /=1000;
			
			GpsDateTime.year =(uint8_t)(pRxBuf[k+10]&0x00ff)<<8 | (uint8_t)pRxBuf[k+9]&0x00ff;
			GpsDateTime.month =(uint8_t)pRxBuf[k+11]&0x00ff;
			GpsDateTime.day =(uint8_t)pRxBuf[k+12]&0x00ff;
						
			GpsDateTime.hour =(uint8_t)pRxBuf[k+13]&0x00ff;
			GpsDateTime.min =(uint8_t)pRxBuf[k+14]&0x00ff;
			GpsDateTime.sec =(uint8_t)pRxBuf[k+15]&0x00ff;
						
			printf("\nUTC TIME %d,%d,%d %d:%d:%d GPS TIME %d\n", GpsDateTime.year,GpsDateTime.month,GpsDateTime.day,GpsDateTime.hour,GpsDateTime.min,GpsDateTime.sec,GpsTow.time);
						
			swiftNavTime |=0x02;
					
			break;
		}
	}	
	
	return swiftNavTime;
}

/*
*|----------------------------------------------------------------------------
*|  Routine: GetOtgDiskSpace
*|  Description:
0*|  Retval:
*|----------------------------------------------------------------------------
*/
unsigned long long GetOtgDiskSpace(void)
{
	unsigned long long result = 0;		
	struct statvfs sfs;
	
	if ( statvfs ( "/home/root/msd/", &sfs) != -1 )
	{	
		result = (unsigned long long)sfs.f_bsize * sfs.f_blocks;
		printf("disk size: %ld\n", result);

		result = (unsigned long long)sfs.f_bsize * sfs.f_bfree;
		printf("disk space available: %ld\n", result);	
	}		
	
	return result;
}

/*
*|----------------------------------------------------------------------------
*|  Routine: main
*|  Description:
0*|  Retval:
*|----------------------------------------------------------------------------
*/
int main(void)
{
	typedef struct
	{
		unsigned int print;	
		unsigned int selftest;
		unsigned int serialComm;
		unsigned int i2cUart;
			
	}TIMER;
	
	TIMER Timer;
	
	int i,j;
	int GpsPositionValid =0;

	static unsigned long NsecTimer=0;
	static unsigned long usecTime=0;
		
	
	static int Toggle =0;
	static struct timeval tv;		

	static char CopyFileString[255];// = asctime(tm);
		  
    time_t s;  // seconds
    struct timespec spec;	

	//char txBuffer[64];
	char rxBuffer[4096];
	uint8_t rxDataBuf[64];
	
	int nbrBytes=0;
	int retval =0;

//	uint16_t LoopCounter =0;	
//	int eventCount =0;
//	int cameraTriggerCount =0;
//	int currentEvCount =0;
//	int currentTrCount =0;	
//	int TestEvCount =0;

    uint8_t SwiftNavTime =0;
		
	double horspd =0.0;
	double trkgnd =0.0;
	
	unsigned long long DiskFreeSpace;
	
	printf(BASE_STATION_VERSION);

	InitGpio();
	printf("GPIO Init\n");
	
	InitPwm();	
	printf("PWM Init\n");
			
	//while(1){}
				
#ifdef TURN_OFF_WIFI	
	/* turn off WiFi */
	system("systemctl stop wpa_supplicant");	
	printf("kill WiFi\n");
#else
	BuzzerChirp(10);
	printf("\n!!!Warning WiFi Enabled!!!\n\n");
#endif	

#ifdef DISABLE_PV_DETECT	
	printf("\n!!!Warning PV Detect Override Enabled!!!\n\n");
#endif

#ifdef SIM_GPS_DATE_TIME	
	printf("\n!!!Warning GPS Data Time Simulation Enabled!!!\n\n");
#endif

#ifdef SWIFTNAV	
	printf("\n!!!Swiftnav GNSS module enabled, baudrate %dbps!!!\n\n", RAW_GPS_BAUD_RATE);
#elif COMNAV	
	printf("\n!!!ComNav GNSS module enabled, baudrate %dbps!!!\n\n", RAW_GPS_BAUD_RATE);
#else
	printf("\n!!!Novatel GNSS module enabled, baudrate %dbps!!!\n\n", RAW_GPS_BAUD_RATE);
#endif
	
#ifdef GPS_DISABLED
	printf("\n!!!Warning GNSS module disabled!!!\n\n");
#endif

	/* UART Settings */
	mraa_uart_context uart;
	uart = mraa_uart_init(0);
	  
	if (uart == NULL)
	{		
		fprintf(stderr, "UART failed to setup\n");
			
		usleep(1000000);
		
		BuzzerOn();
		
		YellowLedOff();
		
		RedLedOn(LED_ON, spec.tv_nsec/1000000);
		
		MainSystem.machState =SYSTEM_STATE_SELF_TEST_FAIL;
		//return EXIT_FAILURE;
	}
	else
	{
		mraa_uart_set_baudrate(uart,RAW_GPS_BAUD_RATE);
		mraa_uart_set_mode(uart,8,MRAA_UART_PARITY_NONE,1);
		mraa_uart_set_flowcontrol(uart,0,0);
		mraa_uart_set_timeout(uart,10,10,1);   

		printf("UART Init\n");	
		
		MainSystem.stateTimer =0;
		
		MainSystem.machState =SYSTEM_STATE_INIT;//SYSTEM_STATE_SELF_TEST_PASS;
	}	
	

	//char txBuffer[] ="Hello Edison 1234567890 Hello Edison 0987654321 Hello Edison 1234567890 Hello Edison 0987654321\n";	
	char txBuffer[] ="LOG TIMEA ONCE\r\n"; // get gps time command
	int MsgCount =0;
	char GnssMsg[10][255];

	strcpy(GnssMsg[0],"log com1 rangecmpb ontime 0.05\r\n");
	strcpy(GnssMsg[6],"LOG TIMEA ONCE\r\n");
	
	printf("start main loop\n");

	
	#ifdef LED_TEST_ENABLED
	printf("\n!!!Warning LED Test Enabled!!!\n\n");
	MainSystem.machState =SYSTEM_STATE_LED_TEST_STATE1;
	#endif

	
	while(1) /* main program loop */
	{
		clock_gettime(CLOCK_REALTIME, &spec);
					
		switch(MainSystem.machState)
		{
			case SYSTEM_STATE_INIT:							
				GpsTow.week =0;
				GpsTow.time =0;
				GpsPositionValid =0;
				SwiftNavTime =0;
				
				GpsOff();
				YellowLedOff();					
				BuzzerOff();				
				PwrDownSignalOff();				

				MainSystem.stateTimer =spec.tv_sec;
				MainSystem.machState =SYSTEM_STATE_STARTUP;			
				break;
			case SYSTEM_STATE_STARTUP:
				if( ReadPowerOnStatus() ==1 ) /* power turned on, let's start */
				{
					RedLedOn(LED_BLINK, spec.tv_nsec/1000000);
					
				#ifdef GPS_DISABLED
					GpsOff();
				#else
					/* power on GNSS receiver */
					GpsOn();
				#endif

					printf("\npwr sw assert\n");						
					
					MainSystem.stateTimer =spec.tv_sec;
					MainSystem.machState =SYSTEM_STATE_IDLE;
				}
				else
				{
					GreenLedOff();
//RedLedOn(LED_ON, spec.tv_nsec/1000000);					
//RedLedOn(LED_BLINK, spec.tv_nsec/1000000);
//RedLedOff();

//GreenLedOn(LED_ON, spec.tv_nsec/1000000);					
//GreenLedOn(LED_BLINK, spec.tv_nsec/1000000);
//GreenLedOff();

//YellowLedOn(LED_ON, spec.tv_nsec/1000000);					
//YellowLedOn(LED_BLINK, spec.tv_nsec/1000000);
//YellowLedOff();
					
					printf("\npwr external USB\n");	
					
					MainSystem.machState =SYSTEM_STATE_USB_POWERED;
				}
				break;
			case SYSTEM_STATE_IDLE:
				if( (spec.tv_sec -MainSystem.stateTimer) >=POWER_UP_TIME )
				{
					/* if GPS powered on and data received, turn Vcc on (staggered) */
					//VccOn();
												
					MainSystem.stateTimer =spec.tv_sec;
					
					MainSystem.machState =SYSTEM_STATE_POWERED;
				}
				break;
			case SYSTEM_STATE_POWERED:													
			#ifndef DISABLE_PV_DETECT			
				if( GpsPositionValid )//  || (spec.tv_sec -MainSystem.stateTimer) >=15 )  //!!!!!! 30
			#endif				
				{						
					YellowLedOff();
					RedLedOn(LED_ON, spec.tv_nsec/1000000);
							
					MainSystem.stateTimer =spec.tv_sec;					
					
					Timer.selftest =spec.tv_sec;
	
				#ifndef SWIFTNAV
					/* get GPS time (tow/sow, time of week) */ 
					strcpy( txBuffer, GnssMsg[6]);
					mraa_uart_flush(uart);
					mraa_uart_write(uart, txBuffer, sizeof(txBuffer)); //32);
					//mraa_uart_flush(uart);
					printf(txBuffer);				
				#else				
					printf("\nGet GPS time\n");
					//printf(txBuffer);	
				#endif				
				
					MainSystem.machState =SYSTEM_STATE_WAIT_GPS_TIME;
				}								
				break;
			case SYSTEM_STATE_WAIT_GPS_TIME:
			#ifdef SIM_GPS_DATE_TIME								
				SimGpsTime();

				SetGpsTime(GpsTow.week, GpsTow.time);
				printf("\nGPS SIM time %s\n%d %d\n", rxBuffer, GpsTow.week, GpsTow.time);		

				CreateLogFile("log");
								
				DiskFreeSpace =GetOtgDiskSpace();				
				
				if( DiskFreeSpace <MIN_DISK_SPACE)
					printf("Drive capacity reaching limit!!!\n");
				else
					printf("Drive capacity OK\n");				

				//SwTriggerOn();									
				BuzzerOn(1);
								
				Timer.selftest =spec.tv_sec;				
				
				MainSystem.machState =SYSTEM_STATE_SELF_TEST;						
			#else	
				#ifndef SWIFTNAV					
				// there is binary data in the rxBuffer due to the raw data coming in as well
				// so replace the binary data in the buffer with ASCII character '~' so as that strstr() will work
				for(k=0; k<sizeof(rxBuffer); k++)
				{
					if( rxBuffer[k] <0x20 || rxBuffer[k] > 0x7f)
						rxBuffer[k] ='~';
				}
				
				char *pch;
								
				pch =strstr(rxBuffer, "TIMEA");
				#else				
				SwiftNavTime |=ParseSwiftNavTime(rxBuffer);
			
				char *pch =0;				
				#endif		
				
				if( pch !=0 || (SwiftNavTime &0x02) ==0x02 )	
				{
					printf("%.150s\n", pch);//rxBuffer);
					
					if( ParseGpsTime(pch) )
					{					
						SetGpsTime(GpsTow.week, GpsTow.time);
						//printf("\nGPS time received %s\n%d %d\n", rxBuffer, GpsTow.week, GpsTow.time);
						//printf("\nGPS time received %d %d\n", GpsTow.week, GpsTow.time);
						
						/* create our raw log file */
						if( CreateLogFile(FileExt) )	
						{	
							DiskFreeSpace =GetOtgDiskSpace();						
							
							if( DiskFreeSpace <MIN_DISK_SPACE)
								printf("Drive capacity reaching limit!!!\n");
							else
								printf("Drive capacity OK\n");							
																		
							BuzzerOn(1);							
							
							Timer.selftest =spec.tv_sec;
							
							MainSystem.machState =SYSTEM_STATE_SELF_TEST;						
						}
						else
						{
							Timer.selftest =spec.tv_sec;
												
							MainSystem.machState =SYSTEM_STATE_SELF_TEST_FAIL;
						}
					}
					else
						MainSystem.machState =SYSTEM_STATE_POWERED;						
				}
				
				if( (spec.tv_sec -Timer.selftest) >=5 )
				{
					/* no message received, try again */
					printf("\nGPS time not received, try again!!!\n");
//					printf("%s %d\n", rxBuffer, nbrBytes);
					MainSystem.machState =SYSTEM_STATE_POWERED;						
				}
			#endif
				break;
			case SYSTEM_STATE_SELF_TEST:						
				if( ReadLowBatStatus() ==1 )
				{
					MainSystem.machState =SYSTEM_STATE_SELF_TEST_FAIL;
				}
				if( DiskFreeSpace <MIN_DISK_SPACE )
				{
					MainSystem.machState =SYSTEM_STATE_SELF_TEST_FAIL;
				}				
				if( (spec.tv_sec -Timer.selftest) >=PRE_FLIGHT_TIMEOUT ) 
				{
					/* good to go */
					MainSystem.machState =SYSTEM_STATE_SELF_TEST_PASS;
				}
				break;
			case SYSTEM_STATE_SELF_TEST_PASS:
				
				BuzzerOff();
												
				/* buzzer chirp (4 beep chirp) */
				BuzzerChirp(4);				

				YellowLedOff();
				
				usleep(1000);
				
				GreenLedOn(LED_BLINK, spec.tv_nsec/1000000);
					
				MainSystem.stateTimer =spec.tv_sec;	
					
				MainSystem.machState =SYSTEM_STATE_READY;
				break;
			case SYSTEM_STATE_READY: /* flight time */
				/* power turned off, let's stop */
				if( ReadPowerOnStatus() ==0 ) 
				{								
					printf("\npwring down\n");					
					MainSystem.stateTimer =spec.tv_sec;

					fp = fopen(filename1,"a");
					//fwrite(eventCount, 1, sizeof(eventCount), fp);
					fprintf(fp, "\nShut Down\n");
					fclose(fp);	
												
				#ifdef USE_OTG_MSD	
					system("umount /home/root/msd");
				#else
				#endif	
							
					printf("\npwr down\n");						
										
					MainSystem.machState =SYSTEM_STATE_POWER_DOWN;								
				}				
				break;			
			case  SYSTEM_STATE_READY_GPS_LOST:
				if( GpsPosValid() ==1 )
				{
					YellowLedOff();
					GreenLedOn(LED_ON, spec.tv_nsec/1000000);
					MainSystem.machState =SYSTEM_STATE_READY;
				}
				break;
			case SYSTEM_STATE_POWER_DOWN:
				YellowLedOn(LED_BLINK, spec.tv_nsec/1000000);
				GpsOff();	
				BuzzerChirp(8);
				
				MainSystem.machState =SYSTEM_STATE_SHUT_DOWN;
				break;
		#if 0
			case SYSTEM_STATE_DONE:
				if( (spec.tv_sec -MainSystem.stateTimer) >=5 ) 
				{
					YellowLedOff();					
					VccOff();
					
					PwrDownSignalOn();
					
					MainSystem.machState =SYSTEM_STATE_SHUT_DOWN;
				}
				break;
		#endif
			case SYSTEM_STATE_SHUT_DOWN:			
				/* do nothing state */
				break;
			case SYSTEM_STATE_USB_POWERED:
				/* do nothing state */
				break;				
			case SYSTEM_STATE_SELF_TEST_FAIL:
				if( ReadPowerOnStatus() ==0 )
				{
					MainSystem.machState =SYSTEM_STATE_INIT;								
				}
				BuzzerChirp(2);							
				break;						
			case SYSTEM_STATE_LED_TEST_STATE1:
				BuzzerOff();
				RedLedOn(LED_BLINK, spec.tv_nsec/1000000);
				
				MainSystem.stateTimer =spec.tv_sec;
				MainSystem.machState =SYSTEM_STATE_LED_TEST_STATE2;
				break;
			case SYSTEM_STATE_LED_TEST_STATE2:	
				if( (spec.tv_sec -MainSystem.stateTimer) >=2 )
				{
					YellowLedOff();										
					RedLedOn(LED_ON, spec.tv_nsec/1000000);
					
					MainSystem.stateTimer =spec.tv_sec;
					MainSystem.machState =SYSTEM_STATE_LED_TEST_STATE3;
				}				
				break;			
			case SYSTEM_STATE_LED_TEST_STATE3:
				if( (spec.tv_sec -MainSystem.stateTimer) >=2 )
				{
					RedLedOff();
					GreenLedOn(LED_BLINK, spec.tv_nsec/1000000);
					
					MainSystem.stateTimer =spec.tv_sec;
					MainSystem.machState =SYSTEM_STATE_LED_TEST_STATE4;
				}				
				break;
			case SYSTEM_STATE_LED_TEST_STATE4:
				if( (spec.tv_sec -MainSystem.stateTimer) >=5 )
				{
					YellowLedOff();
					MainSystem.stateTimer =spec.tv_sec;
					MainSystem.machState =SYSTEM_STATE_LED_TEST_STATE1;
				}				
				break;
				
		} // end switch(machState)
			
	#ifdef FILE_WRITE_TEST	
		/* !!!!!!!!!!!!!!!!!!! for testing !!!!!!!!!!!!!!!!!!! */		
		if( MainSystem.machState >=SYSTEM_STATE_SELF_TEST_PASS &&
			MainSystem.machState < SYSTEM_STATE_POWER_DOWN
		  )
		{			
			usleep(15000);			
	
			fp = fopen(filename1,"ab");	
			
			retval =fwrite(&txBuffer, 1, 512, fp);
			
			fclose(fp);		

			if(retval <=0 )
			{
				printf("write fail\n");
			}
		}
	#endif
	
	#ifdef SERIAL_LOOP_BACK	
		/* !!!!!!!!!!!!!!!!!!! for testing !!!!!!!!!!!!!!!!!!! */
		if( MainSystem.machState >=SYSTEM_STATE_SELF_TEST_PASS )
		{
			mraa_uart_write(uart, txBuffer, 32);//sizeof(txBuffer));
			mraa_uart_flush(uart);
			
			usleep(10000);
		}
	#endif	
		
		nbrBytes=0;
		
		if( mraa_uart_data_available(uart, 10) )					
		{
			nbrBytes =mraa_uart_read(uart, rxBuffer, sizeof(rxBuffer));		
		}

	#ifdef SIM_RAW_DATA		
		nbrBytes = sizeof(rxBuffer);
		strcpy( rxBuffer, "RAW DATA TEST RAW DATA TEST RAW DATA TEST\n");
	#endif
	
		if( nbrBytes >0 )
		{
			//count++;					
			//printf( "%d %d\n", nbrBytes, count);
			//printf( "%s %d\n", rxBuffer, nbrBytes);
			
			fflush( stdout );
			
			if( MainSystem.machState >=SYSTEM_STATE_SELF_TEST_PASS &&
				MainSystem.machState <SYSTEM_STATE_SHUT_DOWN
			  )
			{					
				fp = fopen(filename1,"a");
				
				if( fwrite(&rxBuffer, 1, nbrBytes, fp) !=nbrBytes )
				{
					printf("File write error\n");
					BuzzerChirp(2);
				}
				
				fclose(fp);								
			}
			
			/* reset timer */
			Timer.serialComm =spec.tv_sec;					
		}
		else
		{
			/* handle error here */
			if( MainSystem.machState >=SYSTEM_STATE_SELF_TEST_PASS &&
				(spec.tv_sec-Timer.serialComm) >LOSS_COMM_TIMEOUT
			  )
			{
				//printf("No UART data available\n");				
				#ifndef LED_TEST_ENABLED
				//BuzzerChirp(2);
				#endif
			}
		}		

		memset( rxDataBuf, 0x00, sizeof(rxDataBuf) );
				
		if( spec.tv_sec-Timer.print >=1)
		{			
			/* diagnostic stuff for console */
			//printf("Mach state %d timer %d   PV %d   EvCnt %d   TRCnt %d   sec %d\n", MainSystem.machState, MainSystem.stateTimer, GpsPositionValid, GetEventTriggerCount(), GetCameraTriggerCount(), spec.tv_sec);	
			printf("Mach state %d timer %d   PV %d   sec %d\n", MainSystem.machState, MainSystem.stateTimer, GpsPositionValid, spec.tv_sec);	
			
			Timer.print =spec.tv_sec;
				
			/* read the hardware pin */
			GpsPositionValid = GpsPosValid();		
			
			if( Toggle ==0 )
			{
				Toggle =1;
			}
			else
			{			
				Toggle =0;		
			}
		
			if( ReadLowBatStatus() ==1 &&
			    MainSystem.machState !=SYSTEM_STATE_SHUT_DOWN
			  )
			{
				if( Toggle ==0 )
					BuzzerChirp(1);
				else
					BuzzerChirp(3);
				
				printf("Low Battery\n");
			}
			if( ReadPowerOnStatus() )
				printf("Power On\n");
			else
			{
				printf("Power Off\n");
				
				if(MainSystem.machState !=SYSTEM_STATE_READY &&
				   MainSystem.machState !=SYSTEM_STATE_STARTUP &&
				   MainSystem.machState !=SYSTEM_STATE_USB_POWERED &&
				   MainSystem.machState !=SYSTEM_STATE_SHUT_DOWN
				  )
				{					
					MainSystem.machState =SYSTEM_STATE_POWER_DOWN;
				}
			}
		
		#ifdef SIM_RAW_DATA	
			if( MainSystem.machState >=SYSTEM_STATE_SELF_TEST_PASS )
				SetEventTriggerCount(TestEvCount++);
		#endif
		}	
		
		SetLedState(spec.tv_nsec/1000000);
	
//char c;
//printf("Enter character: ");
//c = getc(stdin);
//printf("Character entered: ");
//putc(c, stdout);	
	}

    mraa_uart_stop(uart);
    mraa_deinit();
	
	if(fp !=0 )
		fclose(fp);
	
	printf("\nDONE\n");

	YellowLedOff();
		
	return 0;
}

//https://github.com/intel-iot-devkit/mraa/tree/master/examples
//https://iotdk.intel.com/docs/mraa/v0.9.5/edison.html              // I/O map
