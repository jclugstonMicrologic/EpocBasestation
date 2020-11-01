#include <mraa.h>
#include <time.h>
#include <string.h>
#include <sys/statvfs.h>
#include <stdio.h>

#include <dirent.h>

#define BASE_STATION_VERSION "BASE STATION v0.16 10/25/2020\n"   

#define SWIFTNAV
#ifdef SWIFTNAV
 #define RAW_GPS_BAUD_RATE  230400 //230400, 115200 for RTK version
#else
 #define RAW_GPS_BAUD_RATE  460800 //921600 
#endif

#define POWER_UP_TIME			30
#define LOSS_COMM_TIMEOUT		5
#define PRE_FLIGHT_TIMEOUT		1 //600

#define POWER_DOWN_ASSSERT_TIME	1 /* how long power down button must be held before accepting it */

#define GNNS_MSGS_COMPLETE		2  

#define AMOUNT_TO_COPY			86400 /* how far back in time to copy to usb (24hrs) */

#define MIN_DISK_SPACE			50000000 /* (50MB of 800MB), if we reach this limit, do not proceed (test fail) */

#define MANAGE_DISK_SPACE	   100000000 /* (100MB of 800MB), if we reach this limit, start deleting oldest files */

/* define these for normal operation */
#define USE_OTG_MSD
#define TURN_OFF_WIFI /* is this necesseray for the base station??? */

/* test defines */
//#define AUTO_START
//#define DISABLE_PV_DETECT
//#define SIM_GPS_DATE_TIME
//#define SKIP_SELF_TEST
//#define SIM_RAW_DATA
//#define FILE_WRITE_TEST
//#define LED_TEST_ENABLED
//#define INIT_GNSS_ENABLED
//#define GPS_DISABLED

//#define STORAGE_DRIVE "/media/storage"
/** Local Constants and Types *************************************************/
enum
{
	LED_OFF =0,
	LED_ON,
	LED_BLINK,
	LED_BLINK2
};

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
	SYSTEM_STATE_DEBOUNCE_POWER_DWN,
	SYSTEM_STATE_READY_GPS_LOST,
	SYSTEM_STATE_POWER_DOWN,
	
	SYSTEM_STATE_FINALIZE_DATA_TRANSFER,	
	SYSTEM_STATE_COPY_FILES,
	SYSTEM_STATE_WAIT_USB_MOUNT,
	SYSTEM_STATE_WAIT_USER_FAIL_ACK,
		
	SYSTEM_STATE_SHUT_DOWN,
	SYSTEM_STATE_USB_POWERED,
	SYSTEM_STATE_LINUX_SHUT_DOWN,
	
	SYSTEM_STATE_POWER_DOWN_NO_FILE,
	
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

char logFileName[128];
char fileCpyStr[255];

time_t GpsEpoch;	
unsigned int TotalFileSize =0;

unsigned long long GetOtgDiskSpace(void);
unsigned long GetFileSize(char *pFileName);


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
*|  Routine: ManageLogfiles
*|  Description:
*|  Retval:
*|----------------------------------------------------------------------------
*/
int ManageLogfiles(int findNewest)
{
	DIR *d;
	char st[80];
	char oldest[80];
    struct dirent *dir;
	int nbrFiles =0;
	
	//system("ls /home/root/msd/ -lt");
	
	struct stat filestat;
	struct stat filestat2;
	time_t epoch =0;
	time_t epoch2 =0;
	struct tm fileTime;    
	int cnt =0;
	unsigned long long DiskFreeSpace;
	
	int  memoryCheck =1;
//	int retval =0;	
//	system("mkdir /home/root/msd");
//	system("mkdir /media/usb");

	//system("umount /home/root/msd");
//	retval = system("mount -o offset=8192 /dev/mmcblk0p9 /home/root/msd");
//	printf("%d\n", retval);

	int fileValid =0;
	
	TotalFileSize =0;	
	
	d = opendir("/home/root/msd");
	
//	while(memoryCheck)
	{
		if (d)
		{		
			while ((dir = readdir(d)) != NULL)
			{
				//printf("%s %d\n", dir->d_name, nbrFiles);
		
				strcpy(st, "/home/root/msd/");	
				strcat(st, dir->d_name);
				
				stat(st,&filestat);
				
				cnt =0;
				int value[6];
				int multiplier =1;
				int digCnt =0;
			
				fileValid =0;
				
				while(1)
				{	
					//printf("%x\n", st[cnt]);		
					//if(st[cnt] =='u' )
					
					if(st[++cnt] == '_')
					{	
						//while(digCnt <6 )
						//{
						cnt++;					
						fileTime.tm_year  =0;					
						while(st[cnt] !='_' )
						{
							fileTime.tm_year =(fileTime.tm_year*multiplier) + (st[cnt]-0x30);
							
							value[digCnt] =(value[digCnt]*multiplier) + (st[cnt]-0x30);
							//printf("%x,", st[cnt]);
							multiplier =10;
							
							cnt++;
						}
						
						/* sanity check */
						if( fileTime.tm_year<2017 || fileTime.tm_year >2100 )
						{
							//fileValid =0;
							//epoch2 =0;
							printf("filename error: ");
							printf("%s %d\n", dir->d_name, nbrFiles);
							break;
						}
						else
						{
							fileValid =1;
							nbrFiles ++;
						}
						
						digCnt ++;
						//}
					#if 1	
						cnt ++;
						fileTime.tm_mon =0;
						while(st[cnt] !='_' )
						{
							fileTime.tm_mon =(fileTime.tm_mon*multiplier) + (st[cnt]-0x30);
							
							//printf("%x\n", st[cnt]);
							multiplier =10;
							
							cnt++;
						}					
						
						cnt ++;
						fileTime.tm_mday =0;
						while(st[cnt] !='_' )
						{
							fileTime.tm_mday =(fileTime.tm_mday*multiplier) + (st[cnt]-0x30);
							
							//printf("%x\n", st[cnt]);
							multiplier =10;
							
							cnt++;
						}										

						cnt ++;
						fileTime.tm_hour =0;
						while(st[cnt] !='_' )
						{
							fileTime.tm_hour =(fileTime.tm_hour*multiplier) + (st[cnt]-0x30);
							
							//printf("%x\n", st[cnt]);
							multiplier =10;
							
							cnt++;
						}	
						
						cnt ++;
						fileTime.tm_min =0;
						while(st[cnt] !='_' )
						{
							fileTime.tm_min =(fileTime.tm_min*multiplier) + (st[cnt]-0x30);
							
							//printf("%x,", st[cnt]);
							multiplier =10;
							
							cnt++;
						}						
						
						cnt ++;
						fileTime.tm_sec =0;
						while(st[cnt] !='.' )
						{
							fileTime.tm_sec =(fileTime.tm_sec*multiplier) + (st[cnt]-0x30);
							
							//printf("%x,", st[cnt]);
							multiplier =10;
							
							cnt++;
						}					
						
						fileTime.tm_year -=1900;					
						fileTime.tm_mon -=1;					
					#else				
						fileTime.tm_year =value[0] -1900;					
						fileTime.tm_mon =value[1]-1;
						fileTime.tm_mday =value[2];
						fileTime.tm_hour =value[3];
						fileTime.tm_min =value[4];
						fileTime.tm_sec =value[5];
					#endif
						//printf("%d %d %d %d %d %d\n", fileTime.tm_year, fileTime.tm_mon, fileTime.tm_mday, fileTime.tm_hour,fileTime.tm_min,fileTime.tm_sec);
						
						epoch = mktime(&fileTime);		 
						printf("%d %s\n", epoch, dir->d_name);
				
						break;
					}	
					
					if( cnt >50 )
					{
						//printf("break %s %d\n", dir->d_name, cnt);
						cnt =0;
						break;
					}
				}

				//if( fileValid ==1 )
				{
					if( (GpsEpoch -epoch)<AMOUNT_TO_COPY &&  (GpsEpoch -epoch)>=0 )
					{					
						TotalFileSize +=GetFileSize(st);
						printf("Total file size %ld %s\n", TotalFileSize, st);
					}				
								
					if(nbrFiles ==1 && fileValid==1 )
					{
						epoch2 =epoch;
						
						strcpy(oldest, "/home/root/msd/");	
						strcat(oldest, dir->d_name);	

						if( findNewest ==0)
							printf("\nthe FIRST oldest file found is %s\n", oldest);						
						else
						{
							GpsEpoch = mktime(&fileTime);	
							printf("\nthe FIRST newest file found is %s\n", oldest);						
						}
					}			
					//double timeDiff =((double)epoch-(double)epoch2);
					time_t timeDiff =(epoch-epoch2);
					printf("Timediff %d ", timeDiff);
					printf("%d ", epoch);
					printf("%d\n", epoch2);
					
					if(findNewest ==0)
					{
						if( timeDiff<0 )// || timeDiff>1000000000 )
						{
							//filestat2.st_mtime =filestat.st_mtime;
							//printf("oldest file %s", ctime(&filestat.st_mtime));
							epoch2 =epoch;
							strcpy(oldest, "/home/root/msd/");	
							strcat(oldest, dir->d_name);
							
							printf("\noldest file is now %s\n", oldest);			
						}			
					}
					else
					{
						if( timeDiff>0 )
						{
							//filestat2.st_mtime =filestat.st_mtime;
							//printf("oldest file %s", ctime(&filestat.st_mtime));
							epoch2 =epoch;
							strcpy(oldest, "/home/root/msd/");	
							strcat(oldest, dir->d_name);
							
							GpsEpoch = mktime(&fileTime);	
							
							printf("\nnewest file is now %s\n", oldest);			
						}									
					}
				}
			}		
			
			if(findNewest ==0)			
				printf("\nTHE OLDEST FILE IS %s\n", oldest);
			else
			{
				printf("\nTHE NEWEST FILE IS %s\n", oldest);
				
				//GpsEpoch = mktime(&fileTime);		 
				printf("GPS Epoch to use %d\n", GpsEpoch);
			}
			
			DiskFreeSpace =GetOtgDiskSpace();	
			
			//printf("\n %ld %ld\n", DiskFreeSpace, MANAGE_DISK_SPACE);
			
			if( DiskFreeSpace<MANAGE_DISK_SPACE) /* 100MB */
			{
				int retval =remove(oldest);			

				if(retval ==0)
				{
					memoryCheck =0;
					printf("deleted file%s %d\n", oldest, retval);
				}
				else 
				{
					memoryCheck =1;
					printf("failed to delete file%s %d\n", oldest, retval);
				}
			}
			else
			{
				memoryCheck =1;
			}
			
			closedir(d);
		}	
	}
	
	return memoryCheck;
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
	struct tm gpsfileTime;  

#ifdef USE_OTG_MSD	
	sprintf(filename1, "/home/root/msd/b_%d_%d_%d_%d_%d_%d.%s",GpsDateTime.year,GpsDateTime.month,GpsDateTime.day,GpsDateTime.hour,GpsDateTime.min,GpsDateTime.sec,pFileExt);	
	//sprintf(filename1, "/media/storage/b_%d_%d_%d_%d_%d_%d.%s",GpsDateTime.year,GpsDateTime.month,GpsDateTime.day,GpsDateTime.hour,GpsDateTime.min,GpsDateTime.sec,pFileExt);	
	sprintf(logFileName, "b_%d_%d_%d_%d_%d_%d.%s",GpsDateTime.year,GpsDateTime.month,GpsDateTime.day,GpsDateTime.hour,GpsDateTime.min,GpsDateTime.sec,pFileExt);		
#else
	// use the SD card
	sprintf(filename1, "/media/sdcard/%d_%d_%d_%d_%d_%d.log",GpsDateTime.year,GpsDateTime.month,GpsDateTime.day,GpsDateTime.hour,GpsDateTime.min,GpsDateTime.sec);	
#endif

	//printf("%s\n", filename1);
	
	gpsfileTime.tm_year =GpsDateTime.year -1900;					
	gpsfileTime.tm_mon =GpsDateTime.month-1;
	gpsfileTime.tm_mday =GpsDateTime.day;
	gpsfileTime.tm_hour =GpsDateTime.hour;
	gpsfileTime.tm_min =GpsDateTime.min;
	gpsfileTime.tm_sec =GpsDateTime.sec;
				
	GpsEpoch = mktime(&gpsfileTime);		 
	printf("%d %s\n", GpsEpoch, filename1);


#ifdef USE_OTG_MSD
	system("mkdir /home/root/msd");
	system("mkdir /media/usb");

	system("umount /home/root/msd");
	
	retval = system("mount -o offset=8192 /dev/mmcblk0p9 /home/root/msd");
	//retval = system("mount -o offset=8192 /dev/mmcblk0p9 /media/storage");
	printf("%d\n", retval);
	
#if 1	
	if(retval !=0 )
	{
		retval = system("mount -o offset=8192 /dev/mmcblk0p9 /home/root/msd");
		printf("%d\n", retval);		
	}
#endif
	
	while( ManageLogfiles(0) ==0 ){}	
	
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
	int value;

	struct timespec spec;	
	clock_gettime(CLOCK_REALTIME, &spec);	
	
	GpsTow.week =1949;
	GpsTow.time =321548;

	GpsDateTime.year =2020;
	
	value =rand() % 12;
	GpsDateTime.month =06;//value;
	
	value =rand() % 30;
	GpsDateTime.day  =22;//value;
	
	value =rand() % 12;
	GpsDateTime.hour =1;//value;
	
	value =spec.tv_sec% 60;
	GpsDateTime.min =value;
	
	value =spec.tv_sec % 60;
	GpsDateTime.sec =value;
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
	
	uint8_t status;
	
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
			status =(uint8_t)(pRxBuf[k+4]); 
			
			printf("\nstatus %d\n", status);
			
			if( (status&0x01) ==0x01 )
			{
				GpsTow.time =(uint8_t)(pRxBuf[k+8]&0x00ff)<<24 | (uint8_t)(pRxBuf[k+7]&0x00ff)<<16 | (uint8_t)(pRxBuf[k+6]&0x00ff)<<8 | (uint8_t)pRxBuf[k+5]&0x00ff;
				GpsTow.time /=1000;
				
				GpsDateTime.year =(uint8_t)(pRxBuf[k+10]&0x00ff)<<8 | (uint8_t)pRxBuf[k+9]&0x00ff;
				GpsDateTime.month =(uint8_t)pRxBuf[k+11]&0x00ff;
				GpsDateTime.day =(uint8_t)pRxBuf[k+12]&0x00ff;
							
				GpsDateTime.hour =(uint8_t)pRxBuf[k+13]&0x00ff;
				GpsDateTime.min =(uint8_t)pRxBuf[k+14]&0x00ff;
				GpsDateTime.sec =(uint8_t)pRxBuf[k+15]&0x00ff;
							
				printf("\nUTC TIME status %d %d,%d,%d %d:%d:%d GPS TIME %d\n", status, GpsDateTime.year,GpsDateTime.month,GpsDateTime.day,GpsDateTime.hour,GpsDateTime.min,GpsDateTime.sec,GpsTow.time);
				
				//if( GpsDateTime.year ==0 || GpsDateTime.month ==0 || GpsDateTime.day ==0 )				
					//swiftNavTime =0x00;				
				//else
				swiftNavTime |=0x02;
			}		
			break;
		}
	}	
	
	return swiftNavTime;
}

/*
*|----------------------------------------------------------------------------
*|  Routine: GetOtgDiskSpace
*|  Description:
*|  Retval:
*|----------------------------------------------------------------------------
*/
unsigned long long GetOtgDiskSpace(void)
{
	unsigned long long result = 0;		
	struct statvfs sfs;
	
	if ( statvfs ( "/home/root/msd/", &sfs) != -1 )
	{	
		result = (unsigned long long)sfs.f_bsize * sfs.f_blocks;
		printf("internal disk size: %ldMB\n", result/1000000);

		result = (unsigned long long)sfs.f_bsize * sfs.f_bfree;
		printf("internal disk space available: %ldMB\n", result/1000000);
	}		
	
	return result;
}


/*
*|----------------------------------------------------------------------------
*|  Routine: GetUsbDiskSpace
*|  Description:
*|  Retval:
*|----------------------------------------------------------------------------
*/
float GetUsbDiskSpace(void)
{
	//unsigned long long result = 0;		
	unsigned long long result = 0;		
	struct statvfs sfs;
	
	float avalue =0.0;
	
	if ( statvfs ("/media/usb/", &sfs) != -1 )
	{	
		//result =(unsigned long long)( (unsigned long long)sfs.f_bsize * (unsigned long long)sfs.f_blocks);
		//result =(unsigned long long)(sfs.f_bsize * sfs.f_blocks);
		
		avalue =(float)((float)sfs.f_bsize/(float)1024.0/1024.0 *sfs.f_blocks);
		//result /=(1024*1024);
		//printf("disk size: %d %ld\n", sfs.f_bsize, sfs.f_blocks);
		printf("disk size: %4.2fMB\n", avalue);

		//result = (unsigned long long)sfs.f_bsize * sfs.f_bfree;
		//result = (unsigned long long)(sfs.f_bsize * sfs.f_bfree);
		
		avalue =(float)((float)sfs.f_bsize/(float)1024.0/1024.0 *sfs.f_bfree);
		//printf("disk space available: %d %d\n",sfs.f_bsize, sfs.f_bfree);
		printf("disk space available: %4.2fMB\n",avalue);
	}		
	
	return avalue; // result;
}

/*
*|----------------------------------------------------------------------------
*|  Routine: MountUsbDrive
*|  Description:
*|  Retval:
*|----------------------------------------------------------------------------
*/
int MountUsbDrive(void)
{
	int retval =0;
	
	/* first umount USB drive */
	system("umount /media/usb");
	
	usleep(500000);
	
	/* now mount USB drive */
	retval = system("mount /dev/sda /media/usb");
	printf("%d\n", retval);
					
	if( retval !=0 )
	{
		usleep(100000);
		retval = system("mount /dev/sda1 /media/usb");
		printf("%d\n", retval);						
	}
				
	if( retval !=0 )
	{
		usleep(100000);
		retval = system("mount /dev/sdb /media/usb");
		printf("%d\n", retval);						
	}	

	if( retval !=0 )
	{
		usleep(100000);
		retval = system("mount /dev/sdb1 /media/usb");
		printf("%d\n", retval);						
	}					

	if( retval !=0 )
	{
		/* failed all attempts, reset USB */
		GpsOff();	
		usleep(1000000);		
			
		GpsOn();
		//RedLedOff(LED_OFF);		
		usleep(1000000);		
		//RedLedOn(LED_ON);
		usleep(1000000);		
		
		usleep(1000000);		
		
		usleep(1000000);	
		
		usleep(1000000);	
	}
	
	return retval;
}

/*
*|----------------------------------------------------------------------------
*|  Routine: CopyLogToSdCard
*|  Description:
*|  Retval:
*|----------------------------------------------------------------------------
*/
void CopyLogToSdCard(void)
{
	/* copy log file to SD card */
	sprintf(fileCpyStr, "cp /home/root/msd/%s /media/sdcard/%s",logFileName,logFileName);
	system(fileCpyStr);
	printf(fileCpyStr);
	printf("\n");		
}

/*
*|----------------------------------------------------------------------------
*|  Routine: GetFileSize
*|  Description:
*|  Retval:
*|----------------------------------------------------------------------------
*/
unsigned long GetFileSize(char *pFileName)
{		
	struct stat st;
	stat(pFileName, &st);
	unsigned long fileSize = st.st_size;
	printf("Log file size %d\n", fileSize);	
		
	return fileSize;
}

/*
*|----------------------------------------------------------------------------
*|  Routine: CopyLogToUsbDrive
*|  Description:
*|  Retval:
*|----------------------------------------------------------------------------
*/
void CopyLogToUsbDrive(void)
{	
	DIR *d;
	char st[80];
	char oldest[80];
    struct dirent *dir;
    d = opendir("/home/root/msd");
	int nbrFiles =0;
	
	//system("ls /home/root/msd/ -lt");
	
	struct stat filestat;
	struct stat filestat2;
	time_t fileEpoch =0;
	//time_t epoch2 =0;
	struct tm fileTime;    
	int cnt =0;
	unsigned long long DiskFreeSpace;
	
	if (d)
	{		
		while ((dir = readdir(d)) != NULL)
		{
			nbrFiles ++;
			//printf("%s %d\n", dir->d_name, nbrFiles);
		
			strcpy(st, "/home/root/msd/");	
			strcat(st, dir->d_name);
				
			stat(st,&filestat);
				
			cnt =0;
			int value[6];
			int multiplier =1;
			int digCnt =0;
			
			fileEpoch =0;
			
			while(1)
			{	
				//printf("%x\n", st[cnt]);		
				//if(st[cnt] =='u' )
					
				/* first convert filename to linux time */ 
				if(st[++cnt] == '_')
				{	
					//while(digCnt <6 )
					//{
					cnt++;					
					fileTime.tm_year  =0;					
					while(st[cnt] !='_' )
					{
						fileTime.tm_year =(fileTime.tm_year*multiplier) + (st[cnt]-0x30);
						
						value[digCnt] =(value[digCnt]*multiplier) + (st[cnt]-0x30);
						//printf("%x\n", st[cnt]);
						multiplier =10;
							
						cnt++;
					}
						
					/* sanity check */
					if( fileTime.tm_year<2017 || fileTime.tm_year >2100 )
					{
						printf("filename error: ");
						printf("%s %d\n", dir->d_name, nbrFiles);
						break;
					}
						
					digCnt ++;
						//}
				#if 1	
					cnt ++;
					fileTime.tm_mon =0;
					while(st[cnt] !='_' )
					{
						fileTime.tm_mon =(fileTime.tm_mon*multiplier) + (st[cnt]-0x30);
						
						//printf("%x\n", st[cnt]);
						multiplier =10;
							
						cnt++;
					}					
						
					cnt ++;
					fileTime.tm_mday =0;
					while(st[cnt] !='_' )
					{
						fileTime.tm_mday =(fileTime.tm_mday*multiplier) + (st[cnt]-0x30);
							
							//printf("%x\n", st[cnt]);
						multiplier =10;
							
						cnt++;
					}										

					cnt ++;
					fileTime.tm_hour =0;
					while(st[cnt] !='_' )
					{
						fileTime.tm_hour =(fileTime.tm_hour*multiplier) + (st[cnt]-0x30);
							
						//printf("%x\n", st[cnt]);
						multiplier =10;
							
						cnt++;
					}	
						
					cnt ++;
					fileTime.tm_min =0;
					while(st[cnt] !='_' )
					{
						fileTime.tm_min =(fileTime.tm_min*multiplier) + (st[cnt]-0x30);
							
						//printf("%x\n", st[cnt]);
						multiplier =10;
							
						cnt++;
					}						
						
					cnt ++;
					fileTime.tm_sec =0;
					while(st[cnt] !='.' )
					{
						fileTime.tm_sec =(fileTime.tm_sec*multiplier) + (st[cnt]-0x30);
						
						//printf("%x\n", st[cnt]);
						multiplier =10;
							
						cnt++;
					}					
						
					fileTime.tm_year -=1900;					
					fileTime.tm_mon -=1;					
				#else				
					fileTime.tm_year =value[0] -1900;					
					fileTime.tm_mon =value[1]-1;
					fileTime.tm_mday =value[2];
					fileTime.tm_hour =value[3];
					fileTime.tm_min =value[4];
					fileTime.tm_sec =value[5];
				#endif
					//printf("%d %d %d %d %d %d\n", fileTime.tm_year, fileTime.tm_mon, fileTime.tm_mday, fileTime.tm_hour,fileTime.tm_min,fileTime.tm_sec);
						
					fileEpoch = mktime(&fileTime);		 
					printf("%d %s\n", fileEpoch, dir->d_name);

					break;
				}	
					
				if( cnt >50 )
				{
					//printf("break %s %d\n", dir->d_name, cnt);
					cnt =0;
					break;
				}
			}
										
			if( (GpsEpoch -fileEpoch)<AMOUNT_TO_COPY && (GpsEpoch -fileEpoch)>=0 )
			{
				//printf("%ld %ld %s\n", GpsEpoch, fileEpoch, dir->d_name);
	
				sprintf(fileCpyStr, "cp /home/root/msd/%s /media/usb/%s",dir->d_name,dir->d_name);
				system(fileCpyStr);
				printf(fileCpyStr);
				printf("\n");						
				
				//sprintf(fileCpyStr, "cp /media/usb/%s /home/root/msd/%s",dir->d_name,dir->d_name);
				//system(fileCpyStr);
				//printf(fileCpyStr);
				//printf("\n");										
			}
		}		
						
		closedir(d);
	}	
}

/*
*|----------------------------------------------------------------------------
*|  Routine: main
*|  Description:
*|  Retval:
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
	static int BuzzerToggle =0;
	
	static struct timeval tv;		

	static char CopyFileString[255];// = asctime(tm);
		  
    time_t s;  // seconds
    struct timespec spec;	

	//char txBuffer[64];
	char rxBuffer[4096];
	uint8_t rxDataBuf[64];
	
	int nbrBytes=0;
	int retval =0;
	int copyTries =0;
//	uint16_t LoopCounter =0;	
//	int eventCount =0;
//	int cameraTriggerCount =0;
//	int currentEvCount =0;
//	int currentTrCount =0;	
//	int TestEvCount =0;

    uint8_t SwiftNavTime =0;
	
	float MemAvail =0.0;
	unsigned long LogFileSize=0;
	
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
					
					//printf("\npwr external USB\n");														
					//MainSystem.machState =SYSTEM_STATE_USB_POWERED;
					
					MainSystem.stateTimer =spec.tv_sec;
					MainSystem.machState =SYSTEM_STATE_POWER_DOWN_NO_FILE;					
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
				if( ReadPowerOnStatus() ==0 ) 					
				{		
					MainSystem.stateTimer =spec.tv_sec;
					MainSystem.machState =SYSTEM_STATE_POWER_DOWN_NO_FILE;
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
				if( ReadPowerOnStatus() ==0 ) 					
				{		
					MainSystem.stateTimer =spec.tv_sec;
					MainSystem.machState =SYSTEM_STATE_POWER_DOWN_NO_FILE;
				}				
				break;
			case SYSTEM_STATE_WAIT_GPS_TIME:
			#ifdef SIM_GPS_DATE_TIME								
				SimGpsTime();

				SetGpsTime(GpsTow.week, GpsTow.time);
				printf("\nGPS SIM time %s\n%d %d\n", rxBuffer, GpsTow.week, GpsTow.time);		

				CreateLogFile("sbp");
								
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
			#ifndef AUTO_START				
				if( ReadPowerOnStatus() ==0 ) 					
				{		
					MainSystem.stateTimer =spec.tv_sec;
					MainSystem.machState =SYSTEM_STATE_DEBOUNCE_POWER_DWN;
				}
			#else
				if( (spec.tv_sec -MainSystem.stateTimer) >15 )	
				{
					MainSystem.stateTimer =spec.tv_sec;
					MainSystem.machState =SYSTEM_STATE_DEBOUNCE_POWER_DWN;				
				}
			#endif					
				break;					
			case SYSTEM_STATE_DEBOUNCE_POWER_DWN:
			case SYSTEM_STATE_POWER_DOWN_NO_FILE:
			#ifndef AUTO_START
				if( ReadPowerOnStatus() ==1 ) // false power down
				{
					MainSystem.machState =SYSTEM_STATE_READY;										
				}
				else if( (spec.tv_sec -MainSystem.stateTimer) >=POWER_DOWN_ASSSERT_TIME ) 				
			#endif					
				{					
					printf("\npowering down\n");					
					MainSystem.stateTimer =spec.tv_sec;

					if( MainSystem.machState ==SYSTEM_STATE_DEBOUNCE_POWER_DWN )
					{
						fp = fopen(filename1,"a");
						//fwrite(eventCount, 1, sizeof(eventCount), fp);			
						fprintf(fp, "\nShut Down\n");
						fclose(fp);	
					}
					else
					{
						/* no file was created, do stuff here */
											
						system("mkdir /home/root/msd");
						system("mkdir /media/usb");
						system("umount /home/root/msd");
	
						retval = system("mount -o offset=8192 /dev/mmcblk0p9 /home/root/msd");	
						
						if(retval !=0 )
						{
							usleep(100000);	
							retval = system("mount -o offset=8192 /dev/mmcblk0p9 /home/root/msd");
							printf("%d\n", retval);		
						}
			
						printf("%d\n", retval);						
						
						/* first find newest file to give us an Epoc value (no GPS at this time) */
						while( ManageLogfiles(1) ==0 ){}	
						
						usleep(100000);	
					}
					
					copyTries =0;
					
					MainSystem.machState =SYSTEM_STATE_COPY_FILES;						
				}
				break;
			case SYSTEM_STATE_COPY_FILES:
				MainSystem.stateTimer =spec.tv_sec;
			
				retval =MountUsbDrive();
										
				if( retval ==0 )				
				{
					YellowLedOff();
						
					/* drive mounted, check available space */
					MemAvail =GetUsbDiskSpace();
						
					LogFileSize =GetFileSize(filename1);
							
					TotalFileSize +=LogFileSize;
					TotalFileSize =TotalFileSize/1000000 +1; // add 1MB for precation
						
					printf("Total file size %ldMB; Mem avail: %4.2fMB\n", TotalFileSize, MemAvail);
												
					if( MemAvail >TotalFileSize )
					{
					#if 1							
						RedLedOn(LED_ON);
						BuzzerChirp(2);
						MainSystem.stateTimer =spec.tv_sec;
						MainSystem.machState =SYSTEM_STATE_WAIT_USB_MOUNT; 
					#else
						/* if space okay, write to disk, blink green led  */						
						GreenLedOn(LED_BLINK);
						
						printf("start copy\n");
						CopyLogToUsbDrive();	
						printf("finish copy\n");												
						
						MainSystem.machState =SYSTEM_STATE_FINALIZE_DATA_TRANSFER; 
					#endif
					}
					else
					{						
						RedLedOn(LED_BLINK);
							
						BuzzerToggle =0;
						BuzzerOn(1);							
						
						printf("No space available\n");		
						/* umount USB drive */
						retval = system("umount /media/usb");
						MainSystem.machState =SYSTEM_STATE_WAIT_USER_FAIL_ACK;		
					}											
				}
				else
				{					
					printf("no usb drive found\n");
					
					RedLedOn(LED_ON);
					
					if( ++copyTries >2 )
						MainSystem.machState =SYSTEM_STATE_POWER_DOWN;							
					else
					{
						usleep(500000);
						MainSystem.machState =SYSTEM_STATE_COPY_FILES;
					}
				}										
				break;	
			case SYSTEM_STATE_WAIT_USB_MOUNT:
				if( (spec.tv_sec-MainSystem.stateTimer) >1 )
				{
					/* if space okay, write to disk, blink green led  */						
					GreenLedOn(LED_BLINK);
							
					printf("start copy\n");
					CopyLogToUsbDrive();	
					printf("finish copy\n");												
						
					/* umount USB drive */
					retval = system("umount /media/usb");
					MainSystem.stateTimer =spec.tv_sec;						
					MainSystem.machState =SYSTEM_STATE_FINALIZE_DATA_TRANSFER; 
				}
				break;
			case SYSTEM_STATE_READY_GPS_LOST:
				if( GpsPosValid() ==1 )
				{
					YellowLedOff();
					GreenLedOn(LED_ON, spec.tv_nsec/1000000);
					MainSystem.machState =SYSTEM_STATE_READY;
				}
				break;
			case SYSTEM_STATE_WAIT_USER_FAIL_ACK:
				if( ReadPowerSw() ==0 ) // power down
				{
				#if 0
					RedLedOn(LED_SOLID);
					
					MainSystem.stateTimer =spec.tv_sec;
					MainSystem.machState =SYSTEM_STATE_POWER_DOWN;		
				#else
					BuzzerOff();
					MainSystem.stateTimer =spec.tv_sec;
					MainSystem.machState =SYSTEM_STATE_COPY_FILES;							
				#endif
				}	
				if( (spec.tv_sec -MainSystem.stateTimer) >=1 ) 
				{
					if(BuzzerToggle ==0)
					{
						BuzzerOff();
						BuzzerToggle =1;						
					}				
					else
					{
						BuzzerOn(1);
						BuzzerToggle =0;
					}
					
					MainSystem.stateTimer =spec.tv_sec;
				}	
				break;	
			case SYSTEM_STATE_FINALIZE_DATA_TRANSFER:
				GreenLedOn(LED_ON);
				MainSystem.stateTimer =spec.tv_sec;
				MainSystem.machState =SYSTEM_STATE_POWER_DOWN;						
				break;
			case SYSTEM_STATE_POWER_DOWN:				
				YellowLedOn(LED_BLINK, spec.tv_nsec/1000000);
				
				//printf("\nsignal pic to power down\n");																					
				//PwrDownSignalOn();
				GpsOff();	
				
				BuzzerChirp(8);
				
				system("umount /home/root/msd");
				
				//system("shutdown now");
				MainSystem.stateTimer =spec.tv_sec;
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
				if( (spec.tv_sec - MainSystem.stateTimer) >4 )
				{	
					printf("\nsignal pic to power down\n");																					
					PwrDownSignalOn();

					/* we are about to lose control of the led so 
					   keep led on during shutdown
					*/
					YellowLedOn(LED_ON);
					
					system("shutdown -H now");
					
					MainSystem.stateTimer =spec.tv_sec;
					MainSystem.machState =SYSTEM_STATE_LINUX_SHUT_DOWN;
				}
				break;
			case SYSTEM_STATE_LINUX_SHUT_DOWN:
				/* do nothing state */		
			#if 0
				if( (spec.tv_sec - MainSystem.stateTimer) >8 )
				{
					YellowLedOff();
					system("shutdown -H now");
				}					
			#endif
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
				MainSystem.machState <SYSTEM_STATE_POWER_DOWN //SYSTEM_STATE_SHUT_DOWN
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
				   MainSystem.machState !=SYSTEM_STATE_IDLE &&
				   MainSystem.machState !=SYSTEM_STATE_USB_POWERED &&
				   MainSystem.machState !=SYSTEM_STATE_SHUT_DOWN &&
				   MainSystem.machState !=SYSTEM_STATE_COPY_FILES &&
				   MainSystem.machState !=SYSTEM_STATE_WAIT_USB_MOUNT &&
				   MainSystem.machState !=SYSTEM_STATE_FINALIZE_DATA_TRANSFER &&
				   MainSystem.machState !=SYSTEM_STATE_WAIT_USER_FAIL_ACK &&
				   MainSystem.machState !=SYSTEM_STATE_LINUX_SHUT_DOWN &&				   				  
				   MainSystem.machState !=SYSTEM_STATE_POWERED
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
