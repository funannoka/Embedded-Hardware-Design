#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#define ADC_NO_OF_SAMPLES 16
#define Vref              3.3
#define Resolution        4095

struct Complex
{	double rl;        //Real Part
    double im;        //Imaginary Part
}  X[33], U, W, T, Tmp;

void FFT(void)
{
    int M = 4;
    int N = pow(2.0, M);
    
    int i = 1, j = 1, k = 1;
    int LE = 0, LE1 = 0;
    int IP = 0;
    
    for (k = 1; k <= M; k++)
    {
        LE = pow(2.0, M + 1 - k);
        LE1 = LE / 2;
        
        U.rl = 1.0;
        U.im = 0.0;
        
        W.rl = cos(M_PI / (double)LE1);
        W.im = -sin(M_PI/ (double)LE1);
        
        for (j = 1; j <= LE1; j++)
        {
            for (i = j; i <= N; i = i + LE)
            {
                IP = i + LE1;
                T.rl = X[i].rl + X[IP].rl;
                T.im = X[i].im + X[IP].im;
                Tmp.rl = X[i].rl - X[IP].rl;
                Tmp.im = X[i].im - X[IP].im;
                X[IP].rl = (Tmp.rl * U.rl) - (Tmp.im * U.im);
                X[IP].im = (Tmp.rl * U.im) + (Tmp.im * U.rl);
                X[i].rl = T.rl;
                X[i].im = T.im;
            }
            Tmp.rl = (U.rl * W.rl) - (U.im * W.im);
            Tmp.im = (U.rl * W.im) + (U.im * W.rl);
            U.rl = Tmp.rl;
            U.im = Tmp.im;
        }
    }
    
    int NV2 = N / 2;
    int NM1 = N - 1;
    int K = 0;
    
    j = 1;
    for (i = 1; i <= NM1; i++)
    {
        if (i >= j) goto TAG25;
        T.rl = X[j].rl;
        T.im = X[j].im;
        
        X[j].rl = X[i].rl;
        X[j].im = X[i].im;
        X[i].rl = T.rl;
        X[i].im = T.im;
    TAG25:	K = NV2;
    TAG26:	if (K >= j) goto TAG30;
        j = j - K;
        K = K / 2;
        goto TAG26;
    TAG30:	j = j + K;
    }
}

void validate_data (float *Volts)
{
    int psd;
    float P[33];
    float mean;
    int i;
    for (i=1; i<=ADC_NO_OF_SAMPLES; i++) {
        X[i].rl = Volts[i];
        X[i].im = 0.0;
    }
    printf("\n\nADC Values:\n");
    for (i=1; i<=ADC_NO_OF_SAMPLES; i++)
        printf("x[%d]:%.2f\n",i,X[i].rl);
    FFT();
    for (i=1; i<=ADC_NO_OF_SAMPLES; i++) {
        X[i].rl = X[i].rl/ADC_NO_OF_SAMPLES;
        X[i].im = X[i].im/ADC_NO_OF_SAMPLES;
    }
    printf("/n/nFFT Values:\n");
    for (i=1; i<=ADC_NO_OF_SAMPLES; i++)
        printf("X[%d]:real == %fimaginary == %f\n",i-1,X[i].rl,X[i].im);
    
    //power spectrum
    printf("\n\n***************Power Spectrum******************\n");
    for (i=1; i<=ADC_NO_OF_SAMPLES; i++) {
        P[i] = pow((X[i].rl),2)+pow((X[i].im),2);
        //P[i]= sqrt(((X[i].rl*X[i].rl)+(X[i].im*X[i].im)));
        printf("P[%d]:%.2f\n",i-1,P[i]);
    }
    psd = (ADC_NO_OF_SAMPLES/2)+1;
    printf("Power Spectrum Density == %f\n",P[psd+1]);
    mean = 0;
    for (i = 4; i<=11; i++) {  //12 -> 19
        mean = mean +P[i];
    }
    mean/=8;
    printf("\n");
    if (mean <(0.05*P[1])) {
        printf("Data is valid.\n");
    }
    else
    {
        printf("Data is not valid.\n");
    }
    
}

int main()
{
	int fd = 0;
	char buffer,buffer2[5];int i=0,j=0;int start=0;
    int Voltage[16];
	float Volts[16];
    	int adcVal_raw[16];
    float volt_comp[16];
    float digital_comp[16];
    	//int psd;
    	//float P[33];
    	//float mean;
	fd= open("/dev/ttySAC1", 0);
	if (fd < 0) {
		fd = open("/dev/ttySAC1", 0);
	}
	if (fd < 0) {
		perror("open device leds");
		exit(1);
	}
	while(j<16)
	{	
		read(fd, &buffer,1);
		if(buffer=='\n')
		{
			start=1;
		}
		while(start==1)
		{
		read(fd, &buffer,1);
		if(buffer!='\n')
		{
			buffer2[i]=buffer;
			i++;
		}
		else
		{
			i=0;
			int voltage=atoi(buffer2);
			Voltage[j]=voltage;
			Volts[j] = (Vref/Resolution)*Voltage[j];
            digital_comp[j] = Voltage[j]+ ((1.3636*Volts[j])-1);
            volt_comp[j] = (Vref/Resolution)*digital_comp;//digital_comp/1240.8636;
            printf("volt_comp: %f\n",volt_comp[j]);
            printf("digital_comp: %f\n",digital_comp[j]);
			printf("%d\n",Voltage[j]);
			printf("%f\n",Volts[j]);
			//printf("string: %s\n",buffer2);
			start=0;
			j++;
		}
		}
		
	}
validate_data (volt_comp);
	//ExitFinal:
    return 0;
	close(fd);
	return 0;
}
