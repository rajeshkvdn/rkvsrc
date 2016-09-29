#include <pthread.h>
#include <stdio.h> /* Standard input/output definitions */
#include <string.h> /* String function definitions */
#include <unistd.h> /* UNIX standard function definitions */
#include <fcntl.h> /* File control definitions */
#include <errno.h> /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <stdlib.h>

#define BAUD B115200
//#define BAUD B9600
#define CMDLEN 7
#define TIME_REQ 0x73
#define TO_BCD(x) x = x/10<<4|x%10
//#define UART_DEBUG_PRINT

struct compar{
	int argcount;
	char argvdev[20];
};

typedef struct _RTC_TIME_T
{
unsigned char sec;
unsigned char min;
unsigned char hour;
unsigned char mday;
unsigned char mon;
unsigned char year;
unsigned char wday;
}RTC_TIME_T;

const unsigned int days[4][12] =
{
    {   0,  31,  60,  91, 121, 152, 182, 213, 244, 274, 305, 335},
    { 366, 397, 425, 456, 486, 517, 547, 578, 609, 639, 670, 700},
    { 731, 762, 790, 821, 851, 882, 912, 943, 974,1004,1035,1065},
    {1096,1127,1155,1186,1216,1247,1277,1308,1339,1369,1400,1430},
};

const unsigned char time_sub_cmd[] = {0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0xcf};
const char *weekday[] = {"SUN","MON","TUE","WED","THU","FRI","SAT"};
pthread_t tid_com, tid_key;
int serial_fd;
struct compar cpar;
unsigned int cur_time;

void* portrw(void *arg);
void* keydetect(void *arg);
void prnchar(unsigned char chr);
int initport(int fd);
int open_port(char *prt);
void cmdformat(void);
void show_schedule(unsigned char *schcmd);
int getResponse(unsigned char *cmdsnd, unsigned char *cmdrsp);
unsigned char* hex_decode(unsigned char *in, int len, unsigned char *out);
void epoch_to_date_time(RTC_TIME_T* date_time,unsigned int epoch);
void ShowTime(RTC_TIME_T tval);

void main(int argc, char *argv[])
{

	int err;
	static int tc;
	unsigned char ch;	
	
	cur_time=0;

        if (argc!=2)
                cmdformat();
	cpar.argcount = argc;
	strcpy(cpar.argvdev, argv[1]);

	err = pthread_create(&tid_com, NULL, &portrw, &cpar);
	if (err != 0)
	{
		printf("\ncan't create thread :[%s]", strerror(err));
		exit(1);
	}
	else
		printf("\n Port Commn Thread created successfully\n");
/*
	err = pthread_create(&tid_key, NULL, &keydetect, NULL);
	if (err != 0)
	{
		printf("\ncan't create thread :[%s]", strerror(err));
		exit (1);	
	}
	else
		printf("\n KeyDetect Thread created successfully\n");
*/
	while(1)
		{
		ch = getchar();
		if(ch == 's')
			prnchar(ch);
		usleep(10000);
		}

}

void* portrw(void *arg)
{
	int n, i, rspInd, rspret, read_buff_size = 0;
	struct compar* cp;
        unsigned char buffer[32], cmdbuff[CMDLEN*2+1], bytebuf[CMDLEN], rspbuf[CMDLEN];
        unsigned char nullrsp[]={0x00,0x00,0x00,0x00,0x00,0x00,0x00};
	
	cp = (struct compar*)arg;

	if (cp->argcount!=2)
		cmdformat();


	printf("\nInside Thread : portrw\n");
	serial_fd = open_port(cp->argvdev);

	if(serial_fd == -1)
		printf("Error opening serial port %s \n", cp->argvdev);
	else
	{
		printf("Serial Port %s is now open \n", cp->argvdev);

		if(initport(serial_fd) == -1)
		{
			printf("Error Initializing port");
			close(serial_fd);
			return 0;
		}

		sleep(.5);

		while(1)
		{
			read_buff_size = 0;
			memset(buffer, 0, sizeof(buffer));
			memset(cmdbuff, 0, sizeof(cmdbuff));
			printf("\nListen UART..........\n");
			usleep(100);
			//                      memset(buffer, 0, sizeof(buffer));
			while(read_buff_size < CMDLEN*2)
			{
				n = read(serial_fd, buffer, CMDLEN*2);
				if (n < 0)
				{
					fputs("read failed!\n", stderr);
					break;
				}
				else
				{	
#ifdef UART_DEBUG_PRINT
					printf("\nSuccessfully read %d bytes from serial port \n", n);
					for(i = 0; i<n ; i++)
						printf("%d ", buffer[i]);
#endif
				/*To clear some garbage values received during CSR power cycle*/
				if(!buffer[0] || (buffer[0] > 0x7F)) 
					{
					sleep(1);
					tcflush(serial_fd, TCIOFLUSH);
					read_buff_size = 0;
					memset(buffer, 0, sizeof(buffer));
					continue;					
					}		
				}
				memcpy(&cmdbuff[read_buff_size], buffer, n);
				read_buff_size += n; 
			memset(buffer, 0, sizeof(buffer));
			}
			usleep(10000);
			tcflush(serial_fd, TCIOFLUSH);
                        hex_decode(cmdbuff, CMDLEN*2, bytebuf);
                        printf("\nWiseUART Data Received:\t");
                        for(i = 0; i<CMDLEN ; i++)
                                        printf("  %d", bytebuf[i]);
                        rspret = getResponse(bytebuf, rspbuf);
                        if(rspret < 0)
                                {
                                printf("\nInvalid Command\n");
                                n = write(serial_fd, nullrsp, 7);
                                if (n < 0) break;
                                }
                        else
                        {
			int i;
#ifdef UART_DEBUG_PRINT
                                printf("\nCommand Is = %x \n", bytebuf[0]);
#endif		
				n = 0;
				for(i = 0 ; i<7 ; i++)
				{
                                if(write(serial_fd, &rspbuf[i], 1) > 0)
				      n++;
				}
                                if (n == 0)
                                        {
                                        printf("\nSend bytes failed!!!\n");
                                        break;
                                        }
                                else
                                        {
                                        printf("\nWiseUART Data Sent:\t");
                                        for(i = 0; i<CMDLEN ; i++)
                                                printf(" %d", rspbuf[i]);
                                        }
                        }
                        //      break;
                }	

	}

	sleep(.5);
	printf("\n\nNow closing Serial Port /dev/ttyUSB0 \n\n");
	close(serial_fd);
	sleep(1);
}


void* keydetect(void *arg)
{
	sleep(0.5);

}

int getResponse(unsigned char *cmdsnd, unsigned char *cmdrsp)
{
int ret = 0;
RTC_TIME_T dt;
memset(cmdrsp, 0x00, 7);
switch(cmdsnd[0])
        {
        case 0x12:      /*Operate ON*/
                cmdrsp[0] = 0x13;
                cmdrsp[1] = cmdsnd[1];
                /*LRC*/
                cmdrsp[6] = 0xFF^cmdrsp[0]^cmdrsp[1];
                break;
        case 0x14:      /*Operate OFF*/
                cmdrsp[0] = 0x15;
                cmdrsp[1] = cmdsnd[1];
                /*LRC*/
                cmdrsp[6] = 0xFF^cmdrsp[0]^cmdrsp[1];
                break;
        case 0x10:      /*Set color*/
                cmdrsp[0] = 0x11;
                cmdrsp[1] = cmdsnd[1];
                cmdrsp[2] = cmdsnd[2];
                cmdrsp[3] = cmdsnd[3];
                cmdrsp[4] = cmdsnd[4];
                cmdrsp[5] = cmdsnd[5];
                /*LRC*/
                cmdrsp[6] = 0xFF^cmdrsp[0]^cmdrsp[1]^cmdrsp[2]^cmdrsp[3]^cmdrsp[4]^cmdrsp[5];
                break;
        case 0x16:      /*Set Schedule*/
                cmdrsp[0] = 0x17;
                cmdrsp[1] = cmdsnd[1];
                cmdrsp[2] = cmdsnd[2];
                cmdrsp[3] = cmdsnd[3];
                cmdrsp[4] = cmdsnd[4];
                cmdrsp[5] = cmdsnd[5];
                /*LRC*/
                cmdrsp[6] = 0xFF^cmdrsp[0]^cmdrsp[1]^cmdrsp[2]^cmdrsp[3]^cmdrsp[4]^cmdrsp[5];
		show_schedule(cmdsnd);
                break;
        case 0x18:      /*Set Schedule*/
                cmdrsp[0] = 0x19;
                cmdrsp[1] = cmdsnd[1];
                /*LRC*/
                cmdrsp[6] = 0xFF^cmdrsp[0]^cmdrsp[1];
                break;
        case 0x31:      /*Set time*/
                printf("\nTime Subscribe Response : OK");
		break;
        case 0x3A:      /*Set time*/
                printf("\nTime Sync Received : OK\n");
/*
		printf("%02x", cmdsnd[3]);
		printf("%02x", cmdsnd[4]);
		printf("%02x", cmdsnd[5]);
		printf("%02x", cmdsnd[6]);
*/
		cur_time = ((cmdsnd[3]<<24) | (cmdsnd[4]<<16) | (cmdsnd[5]<<8) | cmdsnd[6]);
		printf("\n%08x", cur_time);
		epoch_to_date_time(&dt, cur_time);
		ShowTime(dt);	
                break;
        default:        /*invalid command*/
        ret = -1;
        }
return ret;
}


void prnchar(unsigned char chr)
{
	int n;

	printf("\nKeystroke : %02x\n", chr);
	/*Send data  to com*/
	n = write(serial_fd, time_sub_cmd, 1);

	if (n < 0)
	{
		printf("\nSend bytes failed!!!\n");
	}
	else
	{
		printf("Send TIME_SUBSCRIBE %d bytes\n", n);
	}
}

//Initialize serial port
int initport(int fd)
{
        int portstatus = 0;

        struct termios options;
        // Get the current options for the port...
        tcgetattr(fd, &options);
        // Set the baud rates to 115200...
        cfsetispeed(&options, BAUD);
        cfsetospeed(&options, BAUD);
        // Enable the receiver and set local mode...
        options.c_cflag |= (CLOCAL | CREAD);

        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;
        //options.c_cflag |= SerialDataBitsInterp(8);           /* CS8 - Selects 8 data bits */
        options.c_cflag &= ~CRTSCTS;                            // disable hardware flow control
        options.c_iflag &= ~(IXON | IXOFF | IXANY);           // disable XON XOFF (for transmit and receive)
        //options.c_cflag |= CRTSCTS;                     /* enable hardware flow control */


        options.c_cc[VMIN] = 1;     //min carachters to be read
        options.c_cc[VTIME] = 1;    //Time to wait for data (tenths of seconds)
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

        //Set the new options for the port...
        tcflush(fd, TCIFLUSH);
        if (tcsetattr(fd, TCSANOW, &options)==-1)
        {
                perror("On tcsetattr:");
                portstatus = -1;
        }
        else
                portstatus = 1;


        return portstatus;
}

/*
 * 'open_port()' - Open serial port 1.
 *
 * Returns the file descriptor on success or -1 on error.
 */
int open_port(char *prt)
{
        int fd; /* File descriptor for the port */
        fd = open(prt , O_RDWR | O_NOCTTY | O_NDELAY);

        if (fd == -1)
        {
                /*
                 * Could not open the port.
                 */
                perror("open_port:\n");
        }
        else
        {
                fcntl(fd, F_SETFL, 0);

        }

        return (fd);
}

void cmdformat(void)
{

        printf("\nComand format:");
        printf("\nportrw <serial_device>");
        printf("\nEg. portrw /dev/ttyUSB0\n");

        exit(1);
}

unsigned char* hex_decode(unsigned char *in, int len, unsigned char *out)
{
        unsigned int i, t, hn, ln;

        for (t = 0,i = 0; i < len; i+=2,++t) {

                hn = in[i] > '9' ? in[i] - 'a' + 10 : in[i] - '0';
                ln = in[i+1] > '9' ? in[i+1] - 'a' + 10 : in[i+1] - '0';

                out[t] = (hn << 4 ) | ln;
        }

        return out;
}

void epoch_to_date_time(RTC_TIME_T* date_time,unsigned int epoch)
{
    unsigned char years, year, month;
    
    date_time->sec = epoch%60; epoch /= 60;
    date_time->min = epoch%60; epoch /= 60;
    date_time->hour   = epoch%24; epoch /= 24;
    date_time->wday = (4 + epoch)%7;

    years = epoch/(365*4+1)*4; epoch %= 365*4+1;

    for (year=3; year>0; year--)
    {
        if (epoch >= days[year][0])
            break;
    }

    for (month=11; month>0; month--)
    {
        if (epoch >= days[year][month])
            break;
    }

    date_time->year  = years+year-30;
    date_time->mon = month+1;
    date_time->mday   = epoch-days[year][month]+1;
    /*Convert to BCD format to set with RTC*/
    TO_BCD(date_time->sec); 
    TO_BCD(date_time->min);
    TO_BCD(date_time->hour);
    TO_BCD(date_time->year);
    TO_BCD(date_time->mon);
    TO_BCD(date_time->mday);
}

void ShowTime(RTC_TIME_T tval)
{
printf("\n------------------------------------");
printf("%02x",tval.wday);
printf(" ");
printf("%02x",tval.mday);
printf("/");
printf("%02x",tval.mon);
printf("/20");
printf("%02x",tval.year);
printf(" ");
printf("%02x",tval.hour);
printf(":");
printf("%02x",tval.min);
printf(":");
printf("%02x",tval.sec);
}

void show_schedule(unsigned char *schcmd)
{
unsigned char sch_id = 0;
unsigned char sch_dates = 0;
unsigned char sch_on_hh = 0;
unsigned char sch_on_mm = 0;
unsigned char sch_off_hh = 0;
unsigned char sch_off_mm = 0;
int i = 0;

sch_id = schcmd[1] >> 4;
sch_dates = ((schcmd[1]&0x0f) << 3) | ((schcmd[2]&0xe0) >> 5);
sch_on_hh = schcmd[2] & 0x1f;
sch_on_mm = (schcmd[3]&0xfc) >> 2;
sch_off_hh = ((schcmd[4]&0x07)<<2) | ((schcmd[5]&0xc0) >> 6);
sch_off_mm = schcmd[5]&0x3f;

printf("\nSchId: %d", sch_id);
printf("\tDates: %02x ", sch_dates);
sch_dates = sch_dates<<1;
for(i=0 ; i<7 ; i++)
	{
	if((sch_dates << i)&0x80)
		printf("  %s", weekday[i]);
	}
printf("\tStartTime: %02d:%02d", sch_on_hh, sch_on_mm);
printf("\tStopTime: %02d:%02d", sch_off_hh, sch_off_mm);
}
