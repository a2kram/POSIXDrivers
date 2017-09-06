/*
	IMU POSIX Serial Driver
*/

#include <stdio.h>
#include <errno.h>
#include <fcntl.h> 
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include "facility.h"
#include "statuscodes.h"

#define PORT_NAME "/dev/ttyACM0"
#define PORT_BAUDRATE B115200

int main()
{
	int fd;
	int rdlen;
	struct termios tty;
	unsigned char *p;    
	unsigned char buf[80];

	// Open port
	fd = open(PORT_NAME, O_RDWR | O_NOCTTY);

	if(fd < 0)
	{
		printf("IMU_SerialDriver Error: Failed to open port %s, %s\n", PORT_NAME, strerror(errno));
		return -1;
	}

	// Get port attributes
	if(tcgetattr(fd, &tty) < 0) 
	{
        printf("IMU_SerialDriver Error: Failed to get port attributes: %s\n", strerror(errno));
        return -1;
    }

	// Set port baudrate
	cfsetospeed(&tty, (speed_t)PORT_BAUDRATE);
    cfsetispeed(&tty, (speed_t)PORT_BAUDRATE); 

	// Ignore modem controls
	tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE;

	// Select 8-bit characters
    tty.c_cflag |= CS8;

	// No parity bit 
    tty.c_cflag &= ~PARENB;

	// 1 stop bit
    tty.c_cflag &= ~CSTOPB;

	// No HW flowcontrol
    tty.c_cflag &= ~CRTSCTS;

	// Disable SW flowcontrol
	tty.c_iflag &= ~(IXON | IXOFF | IXANY);
	
	/* disable canonical input, disable echo,
	disable visually erase chars,
	disable terminal-generated signals */
	tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	/* disable output processing */
	tty.c_oflag &= ~OPOST;

    // Fetch bytes as they become available
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 5;

	// Set attributes
    if (tcsetattr(fd, TCSANOW, &tty) != 0) 
	{
        printf("IMU_SerialDriver Error: Failed to set port attributes: %s\n", strerror(errno));
        return -1;
    }

	// Continuously read and display buffer input
	while(1)
	{
		rdlen = read(fd, buf, sizeof(buf) - 1);

        if(rdlen > 0) 
		{
            printf("Read %d:", rdlen);

            for(p = buf; rdlen-- > 0; ++p)
            {
				printf(" 0x%x", *p);
			}   

            printf("\n");
        } 
		else if(rdlen < 0) 
		{
            printf("IMU_SerialDriver Error: Failed to read from port - %d: %s\n", rdlen, strerror(errno));
        }
	}

	return STATUS_SUCCESS;
}