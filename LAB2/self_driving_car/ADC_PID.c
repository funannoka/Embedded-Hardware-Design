#include "tasks.hpp"
#include "examples/examples.hpp"
#include <stdio.h>
#include <utilities.h>
#include <io.hpp>
#include "lpc_sys.h"
#include "lpc17xx.h"
#include "uart0_min.h"
#include "semphr.h"
#include <string.h>
#include <algorithm>
#include <storage.hpp>
#include <stdio.h>
#include <stdlib.h>
//#include <unistd.h>
#include <math.h>
#include <stdint.h>
//#include <pid.h>
#include <queue.h>
#include <printf_lib.h>

int once = 1;
#define BUFSIZE 77

#define PI 3.14159265

#define number_IError    4     // number of integration terms for IError[n]
#define Kp               30   // gain proportional
#define Kd               10    // gain derivative
#define Ki               10    // gain integral
#define alpha1           1  // weighting for kp
#define alpha2           1  // weighting for ki
#define alpha3           1  // weighting for kd
#define minError         0// min error limit
#define maxError         180 // max error limit
#define itrtns	 		20
float Error[itrtns+1];           // for error
float DError[itrtns];          // derivative of error
float FDError[itrtns];         // forward difference computation for the derivative of error
float BDError[itrtns+1];         // backward difference computation for the derivative of error
float CDError[itrtns];         // central difference computation for the derivative of error
float IError[itrtns];          // integral of error (squared each individual error)
float sumError[itrtns];        // summation of pid errors
int n = 0;
float n_pwm[itrtns];
float contl[itrtns];
float Ang[itrtns];
float disVe[itrtns];
float SpeedVe;
float h[2]={1,-1};
int i,j;
uint8_t UART2Buffer[BUFSIZE];
uint32_t UART2Count=0;
uint32_t UART2TxEmpty=1;
uint32_t msTcMax;
uint16_t percent;
uint16_t valueNeeded;
int prcnt;
float location[3];

QueueHandle_t coordinates = xQueueCreate(1,sizeof(location));

bool pwm_init(void)
        {
        	u0_dbg_printf("enter init\n");

			LPC_SC->PCONP |= (1 << 6); //PWM1 power/clock control bit
			msTcMax = 960000;
			//Peripheral clock selection for PWM1 12 and 13 bit
			LPC_SC->PCLKSEL0 &= ~(3 <<12); // Clear clock Bits
			LPC_SC->PCLKSEL0 |=  (1 << 12); // CCLK / 1

			//Enable Counters,PWM module
			LPC_PWM1->MCR |= (1<<1);//Reset on PWMMR0, reset TC if it matches MR0
			LPC_PWM1->MR0 =   msTcMax;//set PWM cycle(Ton+Toff)=100)(1400microseconds), Reset on PWMMR0: the PWMTC will be reset if PWMMR0 matches it.
			//uint16_t valueNeeded =   0.1*msTcMax; // set pulse to 10% of total time period
			LPC_PWM1->TCR = (1<<0);
			LPC_PWM1->CTCR &= ~(0xF << 0); //the TC is incremented when the Prescale Counter matches the Prescale Register

			//using PWM1.2, GPIO PIN - 2.1
		    LPC_PINCON->PINSEL4 &= ~(3 <<2); // P2.1 clearing to 00 value
		    LPC_PINCON->PINSEL4 |= (1<<2); // P2.1 setting value to 01 for PWM1.2
		    LPC_PWM1->PCR |=(1<<10);//The PWM2 output enabled.
			u0_dbg_printf("exit init\n");
        	return true;
        }

class PID : public scheduler_task
{
    public:
        PID(uint8_t priority) : scheduler_task("PID", 20000, priority)
        {
            /* Nothing to init */
        }
        bool init(void)
        {
        	return true;
        }
        bool run(void *p)
        {
        	float curloc[3], destloc[3],setpoint[3];

        	while(n<=itrtns)
        		{

        		xQueueReceive(coordinates,curloc,portMAX_DELAY);
        		u0_dbg_printf("\ncurrent coordinates X: %f, Y : %f, Z: %f\n",curloc[0],curloc[1],curloc[2]);

        		if(n == 0)
        		{
        			pwm_init();
        			contl[n] = 0; 
        			Error[n] = 1;
        			Ang[n] = atan(curloc[1]/curloc[0])*(180/PI);

        		}

        		i=n;

        		IError[n]=0;
        		for(j=n;j>=n-3;j--)
        		{
        			if(j>0)
        			IError[n]=IError[n]+(Error[j]*Error[j]);

        		}
        		u0_dbg_printf("IError[%d]=%f\n",n,IError[n]);
        		for(j=n+1;j<=itrtns;j++)
        		{
        			Error[j]=0;
        		}

        		for(j=2;j<=itrtns-1;j++)
        		{
        			h[j]=0;
        		}
        		BDError[n]=0;
        		if(n>0)
        		BDError[n]=BDError[n]+(Error[n]-Error[n-1]);
        		sumError[n] = Kp*alpha1*Error[n]+Ki*alpha2*IError[n]+Kd*alpha3*BDError[n];
        		u0_dbg_printf("Sum of Error at %d is : %f\n",n,sumError[n]);

        		n_pwm[n] = 50;		    										//linear equation from (50,0) and (200,6000)

        		Angc = (sumError[n]/10)*(PI/180);         						//current angle
        		if(atan(curloc[1]/curloc[0])*(180/PI)+Angc < 90)
        		Ang[n] = Angc;         											//change in the angle needed to get the output
        		u0_dbg_printf("The angle with which the servo motor should move %f\n",Ang[n]);

        		disVe[n] = 1.38*sin(Ang[n])*(0.33); 							//current displacement at 3 samples per second
        		if(n-1 >= 0)
        		disVe[n] = disVe[n]+disVe[n-1];
        		u0_dbg_printf("disVe[%d] : %f\n", n, disVe[n]);
        		//u0_dbg_printf("contl[%d] : %f\n", n, contl[n]);
        		u0_dbg_printf("Ang[%d] : %f\n", n, Ang[n]);
        		u0_dbg_printf("n_pwm[%d] : %f\n", n, n_pwm[n]);
        		if(n_pwm[n]>250)
        			n_pwm[n] = 250;
        		msTcMax = 48000000/(n_pwm[n]);
        		prcnt = (Ang[n]*(180/PI))/180*100;								//max angle of servo motor is 180

        		/*Load the duty cycle of servo motor with the required angle*/
        		{
						valueNeeded = (prcnt*10 * msTcMax) / 100;
						LPC_PWM1->LER |= (1 << 2);
						LPC_PWM1->MR2 = valueNeeded;
        		}

        		if(n < itrtns)
        		{
        			if(n>0)
        			Error[n+1] = 1-disVe[n];
        		}
        		u0_dbg_printf("----------------------------------------------Error[%d] : %f\n", n, Error[n]);
        		vTaskDelay(1000);
        		n++;

        	}



        	return true;
        }
};

class UART2 : public scheduler_task
{
    public:
        UART2(uint8_t priority) : scheduler_task("UART", 2000, priority)
        {
            /* Nothing to init */
        }
        bool init(void)
        {
        	LPC_SC->PCONP|=(1<<24);                                                 //Activate UART2 in PCONP register
        	LPC_SC->PCLKSEL1 &=~(3<<16);                                            //Reset the clock
        	LPC_SC->PCLKSEL1 |= (1<<16);                                            //Set clock as 48MHz
        	LPC_UART2->LCR = (1<<7);                                                //Set DLAB bit
        	LPC_UART2->LCR |= (3<<0);                                               //Set 8-bit character length
        	LPC_UART2->DLM=0;                                                       //Set divisor latch MSB as 0
        	LPC_UART2->DLL = (sys_get_cpu_clock() / (16 * 115200))-0.041666667;               //set divisor latch LSB
        	LPC_UART2->LCR &= ~(1<<7);                                              //Reset the DLAB bit
        	LPC_UART2->FCR |=(1<<0);                                                //Enable the FIFO
        	LPC_PINCON->PINSEL4 &=~ (3<<16)|(3<<18);                                //Set P2.8 as TxD2
        	LPC_PINCON->PINSEL4 |= (2<<16)|(2<<18);                                 //Set P2.9 as RxD2
        	LPC_UART2->FCR|=(1<<1)|(1<<2);                                          //Clear all bytes in Rx and Tx FIFO
        	NVIC_EnableIRQ(UART2_IRQn);                                         //Enable IRQ
        	LPC_UART2->IER |= (1<<0)|(1<<1)|(1<<2);                             //Enable RBR, THRE, and Rx Line Status interrupts
            return true;
        }
        bool run(void *p)
        {
        	return true;
        }
};
class Motor_PWM : public scheduler_task
{
    public:
        Motor_PWM(uint8_t priority) : scheduler_task("Motor_PWM", 2000, priority)
        {
            /* Nothing to init */
        }
        bool init(void)
        {
        	printf("enter init\n");
			//msTcMax = 2400000;// for 50Hz frequency assuming a 120MHz clock
			LPC_SC->PCONP |= (1 << 6); //PWM1 power/clock control bit

			//Peripheral clock selection for PWM1 12 and 13 bit
			LPC_SC->PCLKSEL0 &= ~(3 <<12); // Clear clock Bits
			LPC_SC->PCLKSEL0 |=  (1 << 12); // CCLK / 1

			//Enable Counters,PWM module
			LPC_PWM1->MCR |= (1<<1);//Reset on PWMMR0, reset TC if it matches MR0
			LPC_PWM1->MR0 =   msTcMax;//set PWM cycle(Ton+Toff)=100)(1400microseconds), Reset on PWMMR0: the PWMTC will be reset if PWMMR0 matches it.
			//uint16_t valueNeeded =   0.1*msTcMax; // set pulse to 10% of total time period
			LPC_PWM1->TCR = (1<<0);
			LPC_PWM1->CTCR &= ~(0xF << 0); //the TC is incremented when the Prescale Counter matches the Prescale Register

			//using PWM1.2, GPIO PIN - 2.1
			 LPC_PINCON->PINSEL4 &= ~(3 <<2); // P2.1 clearing to 00 value
			 LPC_PINCON->PINSEL4 |= (1<<2); // P2.1 setting value to 01 for PWM1.2
			 LPC_PWM1->PCR |=(1<<10);//The PWM2 output enabled.
			 printf("exit init\n");
        	 return true;
        }
        bool run(void *p)
        {
        	u0_dbg_printf("enter run\n");
  			valueNeeded = (5 * msTcMax) / 100; //for 1 ms
			//valueNeeded = (10 * msTcMax) / 100; //for 2 ms
			LPC_PWM1->LER |= (1 << 2);
			LPC_PWM1->MR2 = valueNeeded;
			vTaskDelay(330);
			valueNeeded = (50 * msTcMax) / 100; //for 2 ms
			LPC_PWM1->LER |= (1 << 2);
			LPC_PWM1->MR2 = valueNeeded;
			vTaskDelay(330);
			u0_dbg_printf("exit run\n");
        	return true;
        }
};
class I2C_Sensor : public scheduler_task
{
    public:
        I2C_Sensor(uint8_t priority) : scheduler_task("I2C_Sensor", 2000, priority)
        {
            /* Nothing to init */
        }
        bool init(void)
        {

            return true;
        }
        bool run(void *p)
        {
        	unsigned char buffer; int count=1; int Addr=0x3C; int X,Y,Z; uint8_t x[2],y[2],z[2];
        	int i,bit;
        	if(once==1)
        	{
        		for (int addr = 2; addr <= 254; addr += 2) {
        		        		            if (I2C2::getInstance().checkDeviceResponse(addr)) {
        		        		                printf("I2C device responded to address %#4x\n", addr);
        		        		                if(addr==0x3C)
        		        		                {
        		        		                	printf("Discovered sensor LSM303");
        		        		                	break;
        		        		                }
        		        		            }
        		        	I2C2::getInstance().writeReg(0x3C, 0x00, 0x08); //Write 0x08 into register 00h to set to 3Hz
        		        	I2C2::getInstance().writeReg(0x3C, 0x02, 0x00); //write 0x00 into register 02h to set continuous conversion
        		        	once=0;
        		}
        	}

        	I2C2::getInstance().readRegisters(Addr, 0x03, &x[0], count);
        	I2C2::getInstance().readRegisters(Addr, 0x04, &x[1], count);
			X=(0xFFFF^((x[0]<<8|x[1])))+1;
			printf("x:%d\n",X);
        	I2C2::getInstance().readRegisters(Addr, 0x05, &y[0], count);
        	I2C2::getInstance().readRegisters(Addr, 0x06, &y[1], count);
        	Y=(0xFFFF^((y[0]<<8|y[1])))+1;
			printf("y:%d\n",Y);
        	I2C2::getInstance().readRegisters(Addr, 0x07, &z[0], count);
        	I2C2::getInstance().readRegisters(Addr, 0x08, &z[1], count);
        	Z=(0xFFFF^((z[0]<<8|z[1])))+1;
        	printf("z:%d\n",Z);
        	location[0] = X;
        	location[1] = Y;
        	location[2] = Z;
        	xQueueSend(coordinates,location,portMAX_DELAY);
        	vTaskDelay(330);

        	return true;
        }
};

class ADC : public scheduler_task
{
    public:
        ADC(uint8_t priority) : scheduler_task("ADC", 2000, priority)
        {
            /* Nothing to init */
        }
        bool init(void)
        {
        	LPC_SC->PCONP|=(1<<12);                       //Enable PCONP register for the ADC
        		LPC_ADC->ADCR|=(1<<21);                       //Enable the PDN bit
        		LPC_ADC->ADCR &= ~(0XFF);                     //Clear the enable bits of all the channels
        		LPC_ADC->ADCR |= (1<<5);                      //Enable conversions for channel 5
        		LPC_SC->PCLKSEL0&=~(3<<24);                   //Clear the system clock selection bits
        		LPC_SC->PCLKSEL0|=(1<<24);                    //Select (system clock/1)=120MHz
        		LPC_ADC->ADCR &= ~(0xFF<<8);                  //Clearing the pre-scalar bits in the ADC Control Register
        		LPC_ADC->ADCR |= (10<<8);                     //Setting the pre-scalar to 11
        		LPC_PINCON->PINSEL3 &= ~(3<<30);              //Clearing the PINSEL for p0.31
        		LPC_PINCON->PINSEL3 |= (3<<30);               //Setting p0.31 to act as an ADC pin
        		return true;
        }
        void UARTSend(char *BufferPtr, uint32_t Length )
        {
        		UART2TxEmpty=1;
        		for(uint32_t i=0;i<Length;i++)                      //Transmit a string of length "Lenghth"
        		{
        			vTaskDelay(1);
        			while ( !(UART2TxEmpty & 0x01) );                 //Wait while the UART2 Tx FIFO is full. Proceeeds when it becomes empty.
        			LPC_UART2->THR = BufferPtr[i];                    //Put the data in the THR register
        			UART2TxEmpty = 0;                                 //Indicate that Tx FIFO is not empty, contains valid data.
        		}                                                   // UART2TxEmpty is made 1 by the interrupt handler when TX FIFO becomes empty
        }
        bool run(void *p)
        {
        	LPC_ADC->ADCR|= (1<<24);
        	char send[5]="";
			while(!(LPC_ADC->ADDR5&(1<<31)));
			uint16_t output = (LPC_ADC->ADDR5>>4)&(0xfff);
			if(output<10)
			{
				sprintf(send, "000%d\n", output);
			}
			else if(output>=10&&output<100)
			{
				sprintf(send, "00%d\n", output);
			}
			else if(output>=100&&output<1000)
			{
				sprintf(send, "0%d\n", output);
			}
			else
			{
				sprintf(send, "%d\n", output);
			}
			u0_dbg_printf("ADC : %s",send);
			//char *send="Hello";
			vTaskDelay(330);
			UARTSend(send, sizeof(send));
        	return true;
        }
};

extern "C"
{
void UART2_IRQHandler(void)                                              //UART Interrupt handler
{
	uint8_t IIRValue, LSRValue;
	uint8_t Dummy;
	IIRValue=LPC_UART2->IIR;                                             //Read the IIR register (Interrupt Identification register)
	IIRValue>>=1;                                                        //Shift right by one bit
	IIRValue &= 0x07;                                                    //perform AND operation on last three bits [3:1] with 1 to read their values
	LSRValue=LPC_UART2->LSR;                                             //Read the LSR register
	if ( LSRValue & ((1<<1)|(1<<2)|(1<<3)|(1<<7)|(1<<4)) )               //Check if the error bits are set in LSRValue
		{
		  Dummy = LPC_UART1->RBR;                                        //Read the RBR register and return. (Only way to clear the error)
		  return;
		}
	if ( LSRValue & (1<<0) )                                             //If there is valid data in the Receive buffer read it and print it
	{
		UART2Buffer[UART2Count] = LPC_UART2->RBR;
		//printf("%c",UART2Buffer[UART2Count]);
		UART2Count++;
		if ( UART2Count == BUFSIZE )
		{
			UART2Count = 0;
		}
	}
	if ( IIRValue == 0x01 )                                              //If THRE interrupt is enabled
	{
		LSRValue = LPC_UART2->LSR;
		if ( LSRValue & (1<<5) )                                    //If the Tx FIFO is empty return UART2TxEmpty=1
		{
			UART2TxEmpty = 1;
		}
		else
		{
			UART2TxEmpty = 0;
		}
	}
}
}
int main()
{
	scheduler_add_task(new terminalTask(PRIORITY_HIGH));
	//scheduler_add_task(new UART2(PRIORITY_LOW));
	//scheduler_add_task(new ADC(PRIORITY_LOW));
	scheduler_add_task(new I2C_Sensor(PRIORITY_MEDIUM));
	scheduler_add_task(new PID(PRIORITY_LOW));
	scheduler_start();
	return -1;
}
