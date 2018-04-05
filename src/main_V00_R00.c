/*
   Based on spidevlib.c - A user-space program to comunicate using spidev.
                Gustavo Zamboni
*/
#include <stdint.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

char buf[64];
char buf2[1024];
int com_serial;
int failcount;

struct spi_ioc_transfer xfer[2];


//////////
// Init SPIdev
//////////
int spi_init(char filename[40]){
        int file;
    __u8    mode, lsb, bits;
    __u32 speed=3180000;

        if ((file = open(filename,O_RDWR)) < 0)
        {
            printf("Failed to open the bus.\n");
            /* ERROR HANDLING; you can check errno to see what went wrong */
        com_serial=0;
            exit(1);
            }

        ///////////////
        // Verifications
        ///////////////
        //possible modes: mode |= SPI_LOOP; mode |= SPI_CPHA; mode |= SPI_CPOL; mode |= SPI_LSB_FIRST; mode |= SPI_CS_HIGH; mode |= SPI_3WIRE; mode |= SPI_NO_CS; mode |= SPI_READY;
        //multiple possibilities using |
            mode = SPI_MODE_0;
            if (ioctl(file, SPI_IOC_WR_MODE, &mode)<0)   {
                perror("can't set spi mode");
                return -ENOENT;
                }


            if (ioctl(file, SPI_IOC_RD_MODE, &mode) < 0)
                {
                perror("SPI rd_mode");
                return -ENOENT;
                }
            if (ioctl(file, SPI_IOC_RD_LSB_FIRST, &lsb) < 0)
                {
                perror("SPI rd_lsb_fist");
                return -ENOENT;
                }
        //sunxi supports only 8 bits
        /*
            if (ioctl(file, SPI_IOC_WR_BITS_PER_WORD, 8)<0)
                {
                perror("can't set bits per word");
                return;
                }
        */
            if (ioctl(file, SPI_IOC_RD_BITS_PER_WORD, &bits) < 0)
                {
                perror("SPI bits_per_word");
                return -ENOENT;
                }

            if (ioctl(file, SPI_IOC_WR_MAX_SPEED_HZ, &speed)<0)
                {
                perror("can't set max speed hz");
                return -ENOENT;
                }

            if (ioctl(file, SPI_IOC_RD_MAX_SPEED_HZ, &speed) < 0)
                {
                perror("SPI max_speed_hz");
                return -ENOENT;
                }

    printf("%s: spi mode %d, %d bits %s per word, %d Hz max\n",filename, mode, bits, lsb ? "(lsb first) " : "", speed);

    //xfer[0].tx_buf = (unsigned long)buf;
    xfer[0].len = 3; /* Length of  command to write*/
    xfer[0].cs_change = 0; /* Keep CS activated */
    xfer[0].delay_usecs = 0, //delay in us
    xfer[0].speed_hz = speed, //speed
    xfer[0].bits_per_word = 8, // bits per word 8

    //xfer[1].rx_buf = (unsigned long) buf2;
    xfer[1].len = 4; /* Length of Data to read */
    xfer[1].cs_change = 0; /* Keep CS activated */
    xfer[1].delay_usecs = 0;
    xfer[1].speed_hz = speed;
    xfer[1].bits_per_word = 8;

    return file;
}



//////////
// Read n bytes from the 2 bytes add1 add2 address
//////////

char * spi_read(int addr, int nbytes, int file){
    int status;

    memset(buf, 0, sizeof buf);
    memset(buf2, 0, sizeof buf2);
    buf[0] = addr;
    xfer[0].tx_buf = (unsigned long)buf;
    xfer[0].len = 1; /* Length of  command to write*/
    xfer[1].rx_buf = (unsigned long) buf2;
    xfer[1].len = nbytes; /* Length of Data to read */
    status = ioctl(file, SPI_IOC_MESSAGE(2), xfer);

    if (status < 0){
		perror("SPI_IOC_MESSAGE");
		printf("SPI_IOC_MESSAGE");
		return NULL;
	}
    printf("\r -- Sent data: %02x \n", buf[0]);
    printf("\r -- Received data: %02x \n", buf2[0]);

    com_serial=1;
    failcount=0;

    return buf2;
}

//////////
// Read n bytes from the 2 bytes add1 add2 address
//////////

char * spi_read_flash(int addr, int nbytes, int file)
    {
    int status;

    memset(buf, 0, sizeof buf);
    memset(buf2, 0, sizeof buf2);
    buf[0] = 0x0b;
    buf[1] = (addr & 0xFF0000) >> 16;
    buf[2] = (addr & 0xFF00) >> 8;
    buf[3] = (addr & 0xF8);
    buf[4] = 0x00;
    xfer[0].tx_buf = (unsigned long)buf;
    xfer[0].len = 5; /* Length of  command to write*/
    xfer[1].rx_buf = (unsigned long) buf2;
    xfer[1].len = nbytes; /* Length of Data to read */
    status = ioctl(file, SPI_IOC_MESSAGE(2), xfer);
    if (status < 0)
        {
        perror("SPI_IOC_MESSAGE");
        return NULL;
        }
    //printf("env: %02x %02x %02x\n", buf[0], buf[1], buf[2]);
    //printf("ret: %02x %02x %02x %02x\n", buf2[0], buf2[1], buf2[2], buf2[3]);

    com_serial=1;
    failcount=0;
    return buf2;
    }

//////////
// Write n bytes int the 2 bytes address add1 add2
//////////
void spi_write(int addr,int nbytes, uint8_t * out_buf, int file)
    {
    unsigned char buf[32];//, buf2[32];
    int status;

    if (nbytes > 31)
        return;
    memcpy(&buf[1], out_buf, nbytes);
    buf[0] = addr;
    xfer[0].tx_buf = (unsigned long)buf;
    xfer[0].len = nbytes+1; /* Length of  command to write*/
    status = ioctl(file, SPI_IOC_MESSAGE(1), xfer);
    if (status < 0)
        {
        perror("SPI_IOC_MESSAGE");
        return;
        }

    com_serial=1;
    failcount=0;
    }

void spi_write_flash(int addr, int sector ,int nbytes,char value[64],int file)
    {
    unsigned char   buffer[68];
    int status;

    memset(buffer, 0, sizeof buffer);
    buf[0] = addr;
    buf[1] = (sector & 0xFF0000) >> 16;
    buf[2] = (sector & 0xFF00) >> 8;
    buf[3] = (sector & 0xF8);
    memcpy(&buf[4], value, 64);
    xfer[0].tx_buf = (unsigned long)buf;
    xfer[0].len = nbytes+4; /* Length of  command to write*/
    status = ioctl(file, SPI_IOC_MESSAGE(1), xfer);
    if (status < 0)
        {
        perror("SPI_IOC_MESSAGE");
        return;
        }

    com_serial=1;
    failcount=0;
    }

int export_gpios(){
    int sysexport, sysdirection;
    sysexport = open("/sys/class/gpio/export", O_WRONLY);
    if (sysexport < 0){
    	printf("\r -- Failed to open /sys/class/gpio/export ... \n");
        return -2;
    }
    //exports

    write(sysexport, "222", 3); // MCU Reset
    write(sysexport, "178", 3); // EzPort CS
    write(sysexport, "190", 3); // MCU SPI CS (must be high when MCU is being reset for SPI to work)
    write(sysexport, "82", 3);
    write(sysexport, "74", 3);
    close(sysexport);

    printf("\r -- Opened /sys/class/gpio/export and written 222/178/190/82/74 ... \n");

    //direction & initial value
    sysdirection = open("/sys/class/gpio/gpio222/direction", O_WRONLY);
    if (sysdirection < 0){
    	printf("\r -- Failed opening gpio 222 ... \n");
        return -2;
    }
    write(sysdirection, "high", 4);
    close(sysdirection);

    sysdirection = open("/sys/class/gpio/gpio178/direction", O_WRONLY);
    if (sysdirection < 0){
    	printf("\r -- Failed opening gpio 178 ... \n");
        return -2;
    }
    write(sysdirection, "high", 4);
    close(sysdirection);

    sysdirection = open("/sys/class/gpio/gpio190/direction", O_WRONLY);
    if (sysdirection < 0){
    	printf("\r -- Failed opening gpio 190 ... \n");
        return -2;
    }
    write(sysdirection, "high", 4);
    close(sysdirection);

    sysdirection = open("/sys/class/gpio/gpio82/direction", O_WRONLY);
    if (sysdirection < 0){
    	printf("\r -- Failed opening gpio 82 ... \n");
        return -2;
    }
    write(sysdirection, "in", 2);
    close(sysdirection);

    sysdirection = open("/sys/class/gpio/gpio74/direction", O_WRONLY);
    if (sysdirection < 0){
    	printf("\r -- Failed opening gpio 74 ... \n");
        return -2;
    }
    write(sysdirection, "in", 2);
    close(sysdirection);

return 0;
}

#define CS_LOW do {buf1[0] = '0';write(cs, buf1, 1);}while(0)
#define CS_HIGH do {buf1[0] = '1';write(cs, buf1, 1);}while(0)

#define CS_APP_LOW do {buf1[0] = '0';write(cs_app, buf1, 1);}while(0)
#define CS_APP_HIGH do {buf1[0] = '1';write(cs_app, buf1, 1);}while(0)

int main(void) {
    char *buffer;
    unsigned char buf1[64];
    //unsigned char buf2[64];
    char rbuf[64];
    int file, cs, binary, rst;
    int cs_app;
    int read_cnt, file_size = 0;
    int i = 0;

    printf("\r -- Aloha from SPIDev V00 R00! \n");

    file = spi_init("/dev/spidev1.2"); //dev
    /*if (export_gpios() < 0) {
        printf("failed to setup GPIOs\n");
        return -2;
    }*/

    /*binary = open("TK1_ADC_V00_R00.bin", O_RDONLY);
    cs = open("/sys/class/gpio/gpio178/value", O_WRONLY);
    cs_app = open("/sys/class/gpio/gpio190/value", O_WRONLY);
    rst = open("/sys/class/gpio/gpio222/value", O_WRONLY);*/

    if ( file < 0) {
        printf("\r -- Failed to open SPI! \n");
        return -2;
    }
    printf("\r -- Opened spidev1.2! \n");

    buffer = spi_read(0x05, 1, file);
    printf("\r -- Buffer: %s\n", buffer);


    /*if ( cs < 0) {
        printf("failed to open /sys/class/gpio/gpio178/value\n");
        return -2;
    }
    if ( rst < 0) {
        printf("failed to open /sys/class/gpio/gpio222/value\n");
        return -2;
    }
    if ( binary < 0) {
        printf("failed to open test.bin\n");
        return -2;
    }
    printf("Entering EzPort mode....\t");*/

    /*buf1[0] = '1';
    write(rst, buf1, 1);
    usleep(100);

    CS_LOW;
    usleep(100);

    buf1[0] = '0';
    write(rst, buf1, 1);
    usleep(100);

    buf1[0] = '1';
    write(rst, buf1, 1);
    usleep(100);

    CS_HIGH;
    usleep(100);

    CS_LOW;
    buffer = spi_read(0x05, 1, file);
    CS_HIGH;
    //printf("EzPort Status Register = 0x%X\n", buffer[0]);

    CS_LOW;
    spi_write(0x06, 0, buf1, file);
    buf1[0] = '1';write(cs, buf1, 1);
    CS_HIGH;

    CS_LOW;
    buffer = spi_read(0x05, 1, file);
    CS_HIGH;
    printf("EzPort Status Register = 0x%X\n", buffer[0]);*/

    /*if (buffer[0] != 0x02) {
        printf("FAIL\n");
        close(binary);
        close(cs);
        close(file);
        close(rst);
        //close(out);
        return 0;
    }

    printf("Done\n");

    CS_LOW;
    buffer = spi_read_flash(0x400, 16, file);
    CS_HIGH;
    printf("FTFL 0x%X%X%X%X%X%X%X%X \r\n",buffer[7],buffer[6],buffer[5],buffer[4],buffer[3],buffer[2],buffer[1],buffer[0]);
    printf("FTFL 0x%X%X%X%X%X%X%X%X \r\n",buffer[15],buffer[14],buffer[13],buffer[12],buffer[11],buffer[10],buffer[9],buffer[8]);

    printf("EzPort Bulk Erase...");
    CS_LOW;
    spi_write(0xc7, 0, buf1, file);
    CS_HIGH;

    buffer[0] = 0x01;
    // Wait for WIP flag to clear
    while (buffer[0] != 0x00) {
        CS_LOW;
        buffer = spi_read(0x05, 1, file);
        buf1[0] = '1';
        write(cs, buf1, 1);
        if (buffer[0] & 0x40) {
            printf("FAIL\n");
            printf("EzPort WEF asserted!\n");
            close(binary);
            close(cs);
            close(file);
            close(rst);
            //close(out);
            return 0;
        }
    }

    printf(" Complete\n");

    printf("EzPort Flashing ...");
    while( (read_cnt = read(binary, rbuf, 64)) > 0){
        // Set WEN bit
        CS_LOW;
        spi_write(0x06,0,buf1,file);
        CS_HIGH;

        // Write flash sector
        CS_LOW;
        spi_write_flash(0x02, 64*i, 64, rbuf, file);
        CS_HIGH;
        memset(rbuf, 0, sizeof(rbuf));
        i++;
        file_size += read_cnt;

        buffer[0] = 0x01;
        // Wait for WIP flag to clear
        while (buffer[0] != 0x00) {
            CS_LOW;
            buffer = spi_read(0x05, 1, file);
            CS_HIGH;
            if (buffer[0] & 0x40) {
                printf("FAIL\n");
                printf("EzPort WEF asserted!\n");
                    close(binary);
                    close(cs);
                    close(file);
                    close(rst);
                    //close(out);
                return 0;
            }
        }
    }

    printf("Done\n");
    printf("EzPort Flash complete, %d bytes wrote.\n", file_size);

    printf("EzPort Flash verification...");
    lseek(binary, 0,SEEK_SET);
    for (int j = 0; j < i; j++) {
        CS_LOW;
        buffer = spi_read_flash(64*j, 64, file);
        CS_HIGH;
        read_cnt = read(binary, rbuf, 64);
        if (memcmp(rbuf, buffer, read_cnt) != 0) {
            printf("FAIL\n");
            printf("Flash verification failed @ 0x%X\n", 64*j);
            return -5;
        }
    }
    printf("Done\n");

    printf("Resetting the MCU....");
    CS_HIGH;
    buf1[0] = '0';
    write(rst, buf1, 1);
    usleep(100);
    buf1[0] = '1';
    write(rst, buf1, 1);
    printf("Done\n");

    close(binary);
    close(cs);
    close(file);
    close(rst);*/
    close(file);
    return 0;
}
