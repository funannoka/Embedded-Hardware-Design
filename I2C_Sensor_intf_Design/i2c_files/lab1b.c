
#include <stdio.h>
#include <fcntl.h>
#include <getopt.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include "24cXX.h"

#define usage_if(a) do { do_usage_if( a , __LINE__); } while(0);
void do_usage_if(int b, int line)
{
    const static char *eeprog_usage = 
        "I2C-24C08(256 bytes) Read/Write Program, ONLY FOR TEST!\n"
        "Base on 'eeprog' by Stefano Barbato (http://codesink.org/eeprog.html)\n"
        "FriendlyARM Computer Tech. 2009\n";
    if(!b)
        return;
    fprintf(stderr, "%s\n[line %d]\n", eeprog_usage, line);
    exit(1);
}


#define die_if(a, msg) do { do_die_if( a , msg, __LINE__); } while(0);
void do_die_if(int b, char* msg, int line)
{
    if(!b)
        return;
    fprintf(stderr, "Error at line %d: %s\n", line, msg);
    fprintf(stderr, "   sysmsg: %s\n", strerror(errno));
    exit(1);
}

int TwosCompConvrt(int i);
int concatenate(int x, int y);
int main(void)
{
    struct eeprom e;
    int n,i,p;
    int drdy,fd2;
    int out_addr;
    int buff[100];
    int *point = buff;
    int x_h_l_m[20];
    int z_h_l_m[20];
    int y_h_l_m[20];
    int xh_dec;
    int xl_dec;
    int zh_dec;
    int zl_dec;
    int yh_dec;
    int yl_dec;
    int x_axis;
    int z_axis;
    int y_axis;
    
    fprintf(stderr, "Open /dev/i2c/0 with 8bit mode\n");
    die_if(eeprom_open("/dev/i2c/0", 0x1E, EEPROM_TYPE_8BIT_ADDR, &e) < 0,
            "unable to open eeprom device file "
            "(check that the file exists and that it's readable)"); //read/write bit is taken care of in driver, therefore addr is 1E
    die_if(eeprom_write_byte(&e, 0x00, 0x88), "write error");//sub address CRA_REG_M 0x00 //(1) Temperature sensor enables (2) 3Hz minimum data output rate config
    die_if(eeprom_write_byte(&e, 0x01, 0x80), "write error");//sub address CRB_REG_M 0x01 //gain config: +4,-4 input field range (gauss)
    die_if(eeprom_write_byte(&e, 0x02, 0x00), "write error");//sub address MR_REG_M 0x02 //continous conversion mode

    fd2 = open("/dev/drdy", 0);
    if (fd2 < 0) {
        perror("open device drdy");
        exit(1);
    }
    for(;;){
        printf("\nPrint New Sensor Values?\n(1) Yes (2) Exit Program\n");
        printf(":::: >>>>  ");
        scanf("%d",&p);
        if(p == 1){
        drdy = ioctl(fd2,0, NULL);
        if (drdy == 2){
            bzero(x_h_l_m,sizeof(x_h_l_m));
            bzero(z_h_l_m,sizeof(z_h_l_m));
            bzero(y_h_l_m,sizeof(y_h_l_m));
            bzero(buff,sizeof(buff));
            point = &buff[0];
            for (i=0, out_addr = 0x03; i<6; i++, out_addr++) {
                die_if((n = eeprom_read_byte(&e, out_addr)) < 0, "read error"); 
                buff[i] = n;
            }
        

        x_h_l_m[0] = buff[0];        
        x_h_l_m[1] = buff[1];
        z_h_l_m[0] = buff[2];
        z_h_l_m[1] = buff[3];
        y_h_l_m[0] = buff[4];
        y_h_l_m[1] = buff[5];
        // x_axis =  concatenate(x_h_l_m[0], x_h_l_m[1]);
        // y_axis =  concatenate(y_h_l_m[0], y_h_l_m[1]);
        // z_axis =  concatenate(z_h_l_m[0], z_h_l_m[1]);
        xh_dec = TwosCompConvrt(x_h_l_m[0]);
        xl_dec = TwosCompConvrt(x_h_l_m[1]);
        zh_dec = TwosCompConvrt(z_h_l_m[0]);
        zl_dec = TwosCompConvrt(z_h_l_m[1]);
        yh_dec = TwosCompConvrt(y_h_l_m[0]);
        yl_dec = TwosCompConvrt(y_h_l_m[1]);

        x_axis =  concatenate(xh_dec, xl_dec);
        y_axis =  concatenate(yh_dec, yl_dec);
        z_axis =  concatenate(zh_dec, zl_dec);

        // printf("\nx axis 2's comp: %d\n",x_axis);
        // printf("z axis 2's comp: %d\n",z_axis);
        // printf("y axis 2's comp: %d\n",y_axis);
        printf("\n\nx axis magnetometer reading: %d\n",x_axis);
        printf("z axis magnetometer reading: %d\n",y_axis);
        printf("y axis magnetometer reading: %d\n",z_axis);
        }
        }
        if (p==2) {
            goto exit;
        }
    }
exit:
    eeprom_close(&e);
    return 0;
}

int TwosCompConvrt(int i)
{
    const int neg_i = (i & (1 << 7)) != 0;
    int i_dec;
    if (neg_i)
        i_dec = i | ~((1 << 8) - 1);
    else
        i_dec = i;
    return i_dec;
}

int concatenate(int x, int y){
    char str1[20];
    char str2[20];

    sprintf(str1,"%d",x);
    sprintf(str2,"%d",y);
    strcat(str1,str2);

    return atoi(str1);
    }