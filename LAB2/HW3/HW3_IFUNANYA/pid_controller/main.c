//
//  main.c
//  pid_controller
//
//  Created by Ifunanya Nnoka on 4/9/17.
//  Copyright © 2017 Ifunanya Nnoka. All rights reserved.
//
/******************************************************************************************/
/******************************************************************************************/
/*********************************************README***************************************/
/*This is the README for A PID Controller Implementation*
 
 CONTENTS OF THE PACKAGE:
 Folder name : pid_controller
 
 main.c
 vehicleModel.h
 makefile
 output (binary)
 
 SYSTEM REQUIREMENTS:
 
 LINUX OPERATING SYSTEM
 GNU COMPILER
 Gnuplot software install (optional to view motion trajectory)
 
 CODE COMPILATION:
 
 >$ make
 
 OUTPUT COMMAND:
 >$ ./output
 
 GNUPLOT
 To view motion trajectory upon program run, install gnuplot:
 Ubuntu: sudo apt-get install gnuplot-x11
 OSX: brew install gnuplot --with-qt
 
 DESCRIPTION
 This progam is designed to control motor/sensor output to meet a desired set target
 Here we want to achieve a Displacement of 1m. To acheive this the motor steering angle is
 gradually adjusted until target is reached, this is done by using a pid controller to monitor and
 adjust the Frequency of the PWM to either increase speed or reduce speed
 
 */

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/errno.h>
#include <sys/resource.h>
#include <sys/types.h>
#include <math.h>
#include "vehicleModel.h"

#define number_IError    4     // number of integration terms for IError[n]
#define Kp               10.0    // gain proportional
#define Kd               10.0    // gain derivative
#define Ki               10.0    // gain integral
#define alpha1           26.0 // weighting for kp
#define alpha2           0.1  // weighting for ki
#define alpha3           0.33// weighting for kd
#define minError        -1000.0// min error limit
#define maxError        1000.0 // max error limit
#define istart 			0
#define iend		    9
#define deltaTime          0.033333
void plot(double y_axis[]);

int main(int argc, const char * argv[]) {
    int n=0;
    int q;
    int a;
    float alphap;
    float alphai;
    float alphad;
    int ResolADC = 1023; //10bit ADC
    double vRef = 3.3;
    double kcentr[3] = {0.5,0,-0.5}; //central difference kernel
    //    float kback[3] = {0,1,-1}; //backward difference kernel
    //   float kforw[3] = {1,-1,0}; //forward difference kernel
    double kGauss[7] = {0.0587,0.0629,0.0656,0.0665,0.0656,0.0629,0.0587};//1x7 gaussian kernel sigma = 6pixels, boundary (-3:3)
    double x[10] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};//{1.0, 1.11, 1.35, 1.20, 4.32, 2.1, 0.11, -2.1, -1.1, 7.32};
    double h[10],hprime[10];
    double Contl[10]; //vehicle output
    double Error[10];        // for error
    double CDError[10];         // central difference computation for the derivative of error
    double IError[10];          // integral of error (squared each individual error)
    double SumError[10];        // summation of pid errors
    double CntrlProp;
    double CntrlDeriv;
    double CntrlIntgr;
    double fpulse[10];
    double N_pwm[10];
    double Ang[10];
    double disVe[10];
    double SpeedVe[10];
    double Noise[10];
    double * ptr;
    float deltaF = 50;// max PWMMOT/10.0;
    double feedback[10];
    
    alphap = alpha1;
    alphai = alpha2;
    alphad = alpha3;
    
    
    fpulse[0] = deltaF;
    N_pwm[0] = fpulse[0];
    // Ang[0] = N_pwm[n]*STPMOT;
    SpeedVe[0] = SPDVHL;
    disVe[0] =SpeedVe[0]*deltaTime; //meters/sampling t=0.03s
    Ang[0] = disVe[0];
    Contl[0] = 0;
    Error[0] = x[0] - Contl[0];
    for (n = istart; n <= iend; n++)
    {
        //Central Difference Derivative Computation
        CDError[n] = 0;
        ptr = &kcentr[1];
        for (a = -1; a <= 1; a++){
            if(((n-a) > iend)||((n-a) < istart))
                CDError[n] += 0;
            else
                CDError[n] += ptr[a]*Error[n-a];
        }
        
        //compute integral
        IError[n] = 0;
        for (q = n; q >= n-3; q--) {
            //Set Error to zero, for No previous condition before start time
            if (q<0)
            {
                Error[q] = 0;
            }
            IError[n] += Error[q]*Error[q];
        }
        
        CntrlProp = alphap * Kp * Error[n];
        CntrlIntgr = alphai * Ki * IError[n];
        CntrlDeriv = alphad * Kd * CDError[n];
        SumError[n] = CntrlProp + CntrlIntgr + CntrlDeriv;
        if (SumError[n] > maxError) {
            SumError[n] = maxError;
        }
        if (SumError[n] < minError) {
            SumError[n] = minError;
        }
        if(n < 9){
            fpulse[n+1] = 0.5*SumError[n];
            N_pwm[n+1] = fpulse[n+1];
            Ang[n+1] = Ang[n]+((N_pwm[n+1]*STPMOT));
            disVe[n+1] = (Ang[n+1]*deltaTime);
        }
        //Set boundaries to not move beyond min/max threshold
        // if (disVe[n] > DSTMAX) {
        //     disVe[n] = DSTMAX;
        // }
        // if (disVe[n] < DSTMIN) {
        //     disVe[n] = DSTMIN;
        //  }
        //ADC Sensor Output
        h[n] = (ResolADC /(float)vRef)*disVe[n];
        
        //Smoothing/Noise Removal using Gaussian Kernel
        hprime[n] = 0;
        ptr = &kGauss[3];
        for (a = -3; a <= 3; a++) { /*1D Convolution*/
            if(((n-a) > iend)||((n-a) < istart))
                hprime[n] += 0;
            else
                hprime[n] += ptr[a]*h[n-a];
        }
        Noise[n] = h[n]- hprime[n];
        feedback[n] =( hprime[n]*(float)vRef)/(float)(ResolADC );
        
        if(n < 9){
            SpeedVe[n+1] = disVe[n+1]/(deltaTime*(n+1));
            // Contl[n+1] = disVe[n];
            Contl[n+1] = disVe[n+1];
            Error[n+1] = x[n+1] - (Contl[n+1]);
        }
    }
    printf("\nPID Controller Computation results:\n\nx[n] = {%0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f}\n",x[0],x[1],x[2],x[3],x[4],x[5],x[6],x[7],x[8],x[9]);
    printf("\nContl[n] = {%0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f}\n",Contl[0],Contl[1],Contl[2],Contl[3],Contl[4],Contl[5],Contl[6],Contl[7],Contl[8],Contl[9]);
    printf("\nError[n] = {%0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f}\n",Error[0],Error[1],Error[2],Error[3],Error[4],Error[5],Error[6],Error[7],Error[8],Error[9]);
    printf("\nCDError[n] = {%0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f}\n",CDError[0],CDError[1],CDError[2],CDError[3],CDError[4],CDError[5],CDError[6],CDError[7],CDError[8],CDError[9]);
    printf("\nIError[n] = {%0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f}\n",IError[0],IError[1],IError[2],IError[3],IError[4],IError[5],IError[6],IError[7],IError[8],IError[9]);
    printf("\nKp = %0.2f; Ki = %0.2f; Kd = %0.2f\n",Kp,Ki,Kd);
    printf("\nalphaP = %0.2f; alphaI = %0.2f; alphaD = %0.2f\n",alpha1,alpha2,alpha3);
    printf("\nSumError[n] = {%0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f}\n",SumError[0],SumError[1],SumError[2],SumError[3],SumError[4],SumError[5],SumError[6],SumError[7],SumError[8],SumError[9]);
    printf("\nfpulse[n] = {%0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f}\n",fpulse[0],fpulse[1],fpulse[2],fpulse[3],fpulse[4],fpulse[5],fpulse[6],fpulse[7],fpulse[8],fpulse[9]);
    printf("\nN_pwm[n] = {%0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f}\n",N_pwm[0],N_pwm[1],N_pwm[2],N_pwm[3],N_pwm[4],N_pwm[5],N_pwm[6],N_pwm[7],N_pwm[8],N_pwm[9]);
    printf("\nAng[n] = {%0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f}\n",Ang[0],Ang[1],Ang[2],Ang[3],Ang[4],Ang[5],Ang[6],Ang[7],Ang[8],Ang[9]);
    printf("\nSpeedVe[n] = {%0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f}\n",SpeedVe[0],SpeedVe[1],SpeedVe[2],SpeedVe[3],SpeedVe[4],SpeedVe[5],SpeedVe[6],SpeedVe[7],SpeedVe[8],SpeedVe[9]);
    printf("\ndisVe[n] = {%0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f}\n",disVe[0],disVe[1],disVe[2],disVe[3],disVe[4],disVe[5],disVe[6],disVe[7],disVe[8],disVe[9]);
    // printf("\nkGauss[n] = {%0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f}\n\n",kGauss[0],kGauss[1],kGauss[2],kGauss[3],kGauss[4],kGauss[5],kGauss[6]);
   	// printf("\nh[n] = {%0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f}\n\n",h[0],h[1],h[2],h[3],h[4],h[5],h[6],h[7],h[8],h[9]);
    //printf("\nNoise[n] = {%0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f}\n\n",Noise[0],Noise[1],Noise[2],Noise[3],Noise[4],Noise[5],Noise[6],Noise[7],Noise[8],Noise[9]);
    //  printf("\nhprime[n] = {%0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f}\n\n",hprime[0],hprime[1],hprime[2],hprime[3],hprime[4],hprime[5],hprime[6],hprime[7],hprime[8],hprime[9]);
    printf("\nError[n+1] = {%0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f,NULL}\n\n",Error[1],Error[2],Error[3],Error[4],Error[5],Error[6],Error[7],Error[8],Error[9]);
    plot(Contl);
    return 0;
}



#define NUM_POINTS 10
#define NUM_COMMANDS 11

void plot(double y_axis[])
{
    char * commandsForGnuplot[] = {"set title \"Load Response\"","set xlabel \"n (Time)\"","set ylabel \"Cntl[n]\"","set xlabel font \"Arial,12\"","set ylabel font \"Arial,12\"","set samples 100","set table \"table_100\"","plot 'data1.txt' smooth csplines","unset table","set key left","plot 'data1.txt' pt 7 t \"Displacement Points\",\"table_100\" w l t \"motion trajectory\", 'data2.txt' title \"target\" smooth csplines"}; //  smooth bezier smooth csplines
    double xvals[NUM_POINTS] = {0,1,2,3,4,5,6,7,8,9};
    double yvals[NUM_POINTS];
    double y2vals[NUM_POINTS] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    
    FILE * fp = fopen("data1.txt", "w");
    FILE * fp2 = fopen("data2.txt", "w");
    
    FILE * gnuplotPipe = popen ("gnuplot -persistent", "w");
    int i,n;
    for (n=0; n< NUM_POINTS; n++) {
        yvals[n] = y_axis[n];
    }
    for (i=0; i < NUM_POINTS; i++)
    {
        fprintf(fp, "%0.2lf %0.2lf \n", xvals[i], yvals[i]); //Write the data to a temporary file
    }
    for (i=0; i < NUM_POINTS; i++)
   	{
        fprintf(fp2, "%0.2lf %0.2lf \n", xvals[i], y2vals[i]); //Write the data to a temporary file
   	}
    for (i=0; i < NUM_COMMANDS; i++)
    {
        fprintf(gnuplotPipe, "%s \n", commandsForGnuplot[i]); //Send commands to gnuplot one by one.
        fflush(gnuplotPipe);
    }
}

