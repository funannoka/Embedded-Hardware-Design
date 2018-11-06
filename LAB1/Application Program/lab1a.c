//
//  lab1a.c
//  
//
//  Created by Ifunanya Nnoka on 3/5/17.
//
//

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#define SWITCH 0
#define LED    1
#define LED_OFF 1
#define LED_ON 2

int main(void)
{
    int io_dev;
    int fd;
    int device_mode;
    int buffer;
    
    fd = open("/dev/con6", 0);
    if (fd < 0) {
        perror("open device con6");
        exit(1);
    }
	for (;;){
begin:  printf("Enter:\n'1' -> input test\n'2' -> output test\n'3' -> Quit program\n");
        printf(":::: >>>>  ");
        scanf("%d",&buffer);
        if (buffer == 1) {
            //test switch
            io_dev = SWITCH;
            int sw_state = ioctl(fd, io_dev, NULL);
            if (sw_state == 2){
                fprintf(stdout,"switch on\n");
            }
            else if(sw_state == 3){
                fprintf(stdout,"switch off\n");
            }
            else {
                printf(stderr,"Error reading port!!!");
                exit(1);
            }
        }
        else if (buffer == 2){
begin_led:  // write led
            io_dev = LED;
            printf("Enter '0'|'1' to off|on LED\n");
            printf(":::: >>>>  ");
            scanf("%d",&buffer);
            //fgets(buffer,2,stdin);
            if (buffer == 1) {
                //turn on led
                device_mode = LED_ON;
                ioctl(fd, io_dev, device_mode);
            }
            else if (buffer == 0){
                //turn off led
                device_mode = LED_OFF;
                ioctl(fd, io_dev, device_mode);
            }
            else {
                printf("Incorrect command!");
                goto begin_led;
            }
        }
 	else if (buffer == 3){
	break;
	}
        else {
            printf("Incorrect command!");
            goto begin;
        }
	}
    close(fd);
    
    return 0;
}

