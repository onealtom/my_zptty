#include <stdio.h>      /*标准输入输出定义*/
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>     /*Unix标准函数定义*/
#include <sys/types.h>  /**/
#include <sys/stat.h>   /**/
#include <fcntl.h>      /*文件控制定义*/
#include <termios.h>    /*PPSIX终端控制定义*/
#include <errno.h>      /*错误号定义*/
#include <getopt.h>
#include <string.h>

#define FALSE 1
#define TRUE 0

char *recchr="We received:\"";

void print_usage();

int speed_arr[] = { 
	B921600, B460800, B230400, B115200, B57600, B38400, B19200, 
	B9600, B4800, B2400, B1200, B300, 
};

int name_arr[] = {
	921600, 460800, 230400, 115200, 57600, 38400,  19200,  
	9600,  4800,  2400,  1200,  300,  
};

void set_speed(int fd, int speed)
{
	int   i;
	int   status;
	struct termios   Opt;
	tcgetattr(fd, &Opt);

	for ( i= 0;  i < sizeof(speed_arr) / sizeof(int);  i++) {
		if  (speed == name_arr[i])	{
			tcflush(fd, TCIOFLUSH);
			cfsetispeed(&Opt, speed_arr[i]);
			cfsetospeed(&Opt, speed_arr[i]);
			status = tcsetattr(fd, TCSANOW, &Opt);
			if  (status != 0)
				perror("tcsetattr fd1");
				return;
		}
		tcflush(fd,TCIOFLUSH);
  	 }

	if (i == 12){
		printf("\tSorry, please set the correct baud rate!\n\n");
		print_usage(stderr, 1);
	}
}
/*
	*@brief   设置串口数据位，停止位和效验位
	*@param  fd     类型  int  打开的串口文件句柄*
	*@param  databits 类型  int 数据位   取值 为 7 或者8*
	*@param  stopbits 类型  int 停止位   取值为 1 或者2*
	*@param  parity  类型  int  效验类型 取值为N,E,O,,S
*/
int set_Parity(int fd,int databits,int stopbits,int parity)
{
	struct termios options;
	if  ( tcgetattr( fd,&options)  !=  0) {
		perror("SetupSerial 1");
		return(FALSE);
	}
	options.c_cflag &= ~CSIZE ;
	switch (databits) /*设置数据位数*/ {
	case 7:
		options.c_cflag |= CS7;
	break;
	case 8:
		options.c_cflag |= CS8;
	break;
	default:
		fprintf(stderr,"Unsupported data size\n");
		return (FALSE);
	}
	
	switch (parity) {
	case 'n':
	case 'N':
		options.c_cflag &= ~PARENB;   /* Clear parity enable */
		options.c_iflag &= ~INPCK;     /* Enable parity checking */
	break;
	case 'o':
	case 'O':
		options.c_cflag |= (PARODD | PARENB);  /* 设置为奇效验*/
		options.c_iflag |= INPCK;             /* Disnable parity checking */
	break;
	case 'e':
	case 'E':
		options.c_cflag |= PARENB;     /* Enable parity */
		options.c_cflag &= ~PARODD;   /* 转换为偶效验*/ 
		options.c_iflag |= INPCK;       /* Disnable parity checking */
	break;
	case 'S':	
	case 's':  /*as no parity*/
		options.c_cflag &= ~PARENB;
		options.c_cflag &= ~CSTOPB;
	break;
	default:
		fprintf(stderr,"Unsupported parity\n");
		return (FALSE);
	}
 	/* 设置停止位*/  
  	switch (stopbits) {
   	case 1:
    	options.c_cflag &= ~CSTOPB;
  	break;
 	case 2:
  		options.c_cflag |= CSTOPB;
  	break;
 	default:
  		fprintf(stderr,"Unsupported stop bits\n");
  		return (FALSE);
 	}
  	/* Set input parity option */
  	if (parity != 'n')
    	options.c_iflag |= INPCK;
  	options.c_cc[VTIME] = 150; // 15 seconds
    	options.c_cc[VMIN] = 0;

	options.c_lflag &= ~(ECHO | ICANON);

  	tcflush(fd,TCIFLUSH); /* Update the options and do it NOW */
  	if (tcsetattr(fd,TCSANOW,&options) != 0) {
    	perror("SetupSerial 3");
  		return (FALSE);
 	}
	return (TRUE);
}

/**
	*@breif 打开串口
*/
int OpenDev(char *Dev)
{
	int fd = open( Dev, O_RDWR );         //| O_NOCTTY | O_NDELAY
 	if (-1 == fd) { /*设置数据位数*/
   		perror("Can't Open Serial Port");
   		return -1;
	} else
		return fd;
}


/* The name of this program */
const char * program_name;

/* Prints usage information for this program to STREAM (typically
 * stdout or stderr), and exit the program with EXIT_CODE. Does not
 * return.
 */

void print_usage (FILE *stream, int exit_code)
{
    fprintf(stream, "Usage: %s option [ dev... ] \n", program_name);
    fprintf(stream,
            "\t-h  --help     Display this usage information.\n"
            "\t-d  --device   The device ttyS[0-3] or ttySCMA[0-1]\n"
	    "\t-b  --baudrate Set the baud rate you can select\n" 
	    "\t               [230400, 115200, 57600, 38400, 19200, 9600, 4800, 2400, 1200, 300]\n"
            "\t-s  --string   Write the device data\n");
    exit(exit_code);
}


void hexdump(const void *_data, size_t size) {
	char lt = 0x2E;
	char text[17] ;
	/*我们要hexdump的内容的起始地址由*_data指定，大小为size*/ 
	const uint8_t *data = (const uint8_t *)_data;
	/*获取起始地址位置*/
	size_t offset = 0;
	/*偏移量初始化为0，也就是第一行最左边将显示0x0000*/
	while (offset < size) {
		printf("\r0x%04X  ", offset);
		/*0xx  以四位十六进制的方式显示偏移量，如果不足四位的在左边补零，如0x0000--0x0010*/  
		  size_t n = size - offset;
		if (n > 16) {
			n = 16;
		}

		for (size_t i = 0; i < 16; ++i) {
			if (i == 8) {
			printf(" ");
			}

			if (offset + i < size) {
				printf("%02X ", data[offset + i]);  /*x以两位十六进制的方式输出内容*/
			} else {
				printf("   "); /*如果数据已经不足16个，则以空格表示，以便对齐*/
			}
		}
		/*for循环又来输出中间部分十六进制显示的内容，不多于16个一行，8个和8个之间空两格*/
		printf(" ");

		for (size_t i = 0; i < n; ++i) {
			if ( (data[offset + i]>=32) && (data[offset + i]<=126) ) {//ASCII范围
				printf("%c", data[offset + i]);
			} else {
				printf("%c",lt);
			}
		}

		/*%c以字符的形式输出内容，如果是能够显示的字符，则显示，否则以 . 代替*/
		/*每行只显示十六个字节*/
		offset += 16;
		printf("\n");
	}
	printf("\n");
}

/*
	*@breif  main()
 */
int main(int argc, char *argv[])
{
	int  fd, next_option, havearg = 0;
	char *device;
	int i=0,j=0;
	int nread;			/* Read the counts of data */
	char buff[512];		/* Recvice data buffer */
	pid_t pid;
	char *xmit = "1234567890"; /* Default send data */ 
	int speed ;

	sleep(1);
	fd = OpenDev("/dev/zptty0");

	if (fd > 0) {
		set_speed(fd, 115200);
	} else {
		fprintf(stderr, "Error opening %s: %s\n", device, strerror(errno));
		exit(1);
	}

	if (set_Parity(fd,8,1,'N')== FALSE) {
		fprintf(stderr, "Set Parity Error\n");
		close(fd);
		exit(1);
	}

	//pid = fork();	
//
	//if (pid < 0) { 
	//	fprintf(stderr, "Error in fork!\n"); 
	//} else if (pid == 0){
	//	printf("forked\n"); 
	//	while(1) {
	//		 printf("1\n"); 
	//		//printf("%s SEND: %s\n",device, xmit);
	//		//write(fd, xmit, strlen(xmit));
	//		sleep(1);
	//		i++;
	//	}
	//	exit(0);
	//} else { 
	//	printf("forked2\n"); 
	//	while(1) {
	//		nread = read(fd, buff, sizeof(buff));
	//		if (nread > 0) {
	//			buff[nread] = '\0';
	//			printf("%s RECV %d total\n", device, nread);
	//			printf("%s RECV: %s\n", device, buff);
	//		}
	//	}	
	//}

	while(1) {
		sleep(1);
		nread = read(fd, buff, sizeof(buff));
		if (nread > 0) {
			//buff[nread] = '\0';
			printf("%s RECV %d total\n", device, nread);
			hexdump(buff,nread);
			memset(buff,0,nread);

		}
	}
	close(fd);
	exit(0);
}

