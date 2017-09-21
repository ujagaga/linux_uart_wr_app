/*
*   uart_test.c
*   An example application to test linux uart write and read
*   Author: Rada Berar
*   e-mail: ujagaga@gmail.com
*	web: www.radinaradionica.com
*/

#include <errno.h>
#include <fcntl.h> 
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <stdint.h>

/* ----Forward function declarations--------------- */
int uart_setup(char* portname, int speed);
void display_command_list( void );
int get_baud(int baud);
int uart_read(int device, uint8_t* rxBuffer, int length);
int uart_write(int device, uint8_t* txBuffer, int length);

/* ----global variables--------------- */

/*
* uart_setup
* Sets up the uart for requested baud rate. the rest of the parameters are hardcoded.
*
* @param device     id of the hw device
* @param speed      baud rate of the serial port to setup
* @return           0 if successful, 1 if not.
*/
int uart_setup(char* portname, int speed)
{
    struct termios tty;
    int device;

    device = open(portname, O_RDWR | O_NOCTTY | O_SYNC);

    if (device < 0) {
        printf("Error opening %s: %s\n", portname, strerror(errno));
        device = -1;
    }else{

        if (tcgetattr(device, &tty) < 0) {
            printf("Error from tcgetattr: %s\n", strerror(errno));
            device = -1;
        }else{

            cfsetospeed(&tty, (speed_t)speed);
            cfsetispeed(&tty, (speed_t)speed);

            tty.c_cflag |= (CLOCAL | CREAD);    /* ignore modem controls */
            tty.c_cflag &= ~CSIZE;
            tty.c_cflag |= CS8;         /* 8-bit characters */
            tty.c_cflag &= ~PARENB;     /* no parity bit */
            tty.c_cflag &= ~CSTOPB;     /* only need 1 stop bit */
            tty.c_cflag &= ~CRTSCTS;    /* no hardware flowcontrol */

            /* setup for non-canonical mode */
            tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
            tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
            tty.c_oflag &= ~OPOST;
            
            tty.c_cc[VMIN] = 0;         /* fetch bytes as they become available */
            tty.c_cc[VTIME] = 5;        /* half second timer */

            if (tcsetattr(device, TCSANOW, &tty) != 0) {
                printf("Error from tcsetattr: %s\n", strerror(errno));
                device = -1;
            }
        }
    }

    return device;
}

/*
* get_baud
* Converts integer baud to Linux define.
*
* @param baud       requested baud rate
* @return           predefined value for the requested baud rate
*/
int get_baud(int baud)
{
    int result;

	switch (baud) {
	case 9600:
        result = B9600;
        break;
	case 19200:
        result = B19200;
        break;
	case 38400:
        result = B38400;
        break;
	case 57600:
        result = B57600;
        break;
	case 115200:
        result = B115200;
        break;
	case 230400:
        result = B230400;
        break;
	case 460800:
        result = B460800;
        break;
	case 500000:
        result = B500000;
        break;
	case 576000:
        result = B576000;
        break;
	case 921600:
        result = B921600;
        break;
	case 1000000:
        result = B1000000;
        break;
	case 1152000:
        result = B1152000;
        break;
	case 1500000:
        result = B1500000;
        break;
	case 2000000:
        result = B2000000;
        break;
	case 2500000:
        result = B2500000;
        break;
	case 3000000:
        result = B3000000;
        break;
	case 3500000:
        result = B3500000;
        break;
	case 4000000:
        result = B4000000;
        break;
	default: 
        result = B9600;
    }
    
    return result;
}

/*
* uart_read
* Reads the requested number of characters from uart if available 
*
* @param device     Hw ID of the uart device to read from
* @param rxBuffer   pointer to receive buffer
* @param length     maximum number of characters to read
* @return           number of characters received
*/
int uart_read(int device, uint8_t* rxBuffer, int length){
    
    int total = 0;
    int rdlen;

    do {  
        rdlen = read(device, &rxBuffer[total], 1);
        if (rdlen > 0) {
            total += rdlen;           
        }
        /* repeat read to get full message */
        
    } while((total < length) && (rdlen > 0));

    if (rdlen < 0) {
        printf("Error from read: %d: %s\n", rdlen, strerror(errno));
    }

    return total;
}

/*
* uart_write
* Write the requested number of characters to uart
*
* @param device     Hw ID of the uart device to read from
* @param txBuffer   pointer to message to send
* @param length     number of characters to write
* @return           0 if successful, 1 otherwise
*/
int uart_write(int device, uint8_t* txBuffer, int length){
    int wlen;
    int result = EXIT_SUCCESS;

    wlen = write(device, txBuffer, length);
    if (wlen != length) {
        printf("Error from write: %d, %d\n", wlen, errno);
        result = EXIT_FAILURE;
    }
    tcdrain(device);    /* delay for output */

    return result;
}

int main(int argc, char *argv[])
{   
    char *portname = "/dev/ttyUSB0";
    int baudRate = 115200;
    int device;
    int wlen;

    setbuf(stdout, NULL);   // disable output buffering so we can print whatever we want immediatelly

        
    /*baudrate 115200, 8 bits, no parity, 1 stop bit */
    device = uart_setup(portname, get_baud(baudRate));
    if(device < 0){
        return EXIT_FAILURE;
    }
    
    /* simple output */
    char msg[] = " \nNXT test app start\n";
    if(EXIT_SUCCESS != uart_write(device, msg, sizeof(msg))){
        printf("\n Write error!\n");
    }
        
    /* Start reading */
    do {
        uint8_t rxBuffer[256];
        int rdlen;

        rdlen = uart_read(device, rxBuffer, 4);
        
        if (rdlen > 0) {
            int i;
            printf("\n");
            
            for (i = 0; i < rdlen; i++){
                printf(" 0x%x", rxBuffer[i]);
            }
            printf("\n");            

        }/* else timed out or les likely an error*/  
        
        /* repeat read to get full message */
    } while (1);

	return EXIT_SUCCESS;
}