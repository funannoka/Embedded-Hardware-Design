//
//  main.c
//  derivative_calc
//
//  Created by Ifunanya Nnoka on 4/8/17.
//  Copyright © 2017 Ifunanya Nnoka. All rights reserved.
//
/******************************************************************************************/
/******************************************************************************************/
/*********************************************README***************************************/
/*This is the README for A Derivative Calculator Implementation*
 
 CONTENTS OF THE PACKAGE:
 Folder name : derivative_calc
 
 main.c
 makefile
 output (binary)
 
 SYSTEM REQUIREMENTS:
 
 LINUX OPERATING SYSTEM
 GNU COMPILER
 
 CODE COMPILATION:
 
 >$ make
 
 OUTPUT COMMAND:
 >$ ./output
 
 DESCRIPTION
 This program is designed to derive derivates by 1D convolution using central difference, forward difference,
 and backward difference. This is achieved by generating a unique kernel for each method, and doing a convolution 
 of desired function around the kernel. Shifting, then multiplying the function by the kernel and then adding are
 3 procedures involved
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

#define istart 0
#define iend 9

int main(int argc, const char * argv[]) {
    int mthd;
    int i;
    int a;
    double h[10];
    double f[10] = {1.0, 1.11, 1.35, 1.20, 4.32, 2.1, 0.11, -2.1, -1.1, 7.32};
    double kforw[3] = {1,-1,0}; //forward difference kernel
    double kback[3] = {0,1,-1}; //backward difference kernel
    double kcentr[3] = {0.5,0,-0.5}; //central difference kernel
    char DerMthd[50];
    double * ptr;
    double * p;
begin:
    printf("\nSelect approximation method:\n(1) Forward Difference \n(2) Backward Difference \n(3) Central Difference \n(4) Quit\n");
    printf("\n:::: >>>>  ");
    scanf("%d",&mthd);
    
    switch (mthd) {
        case 1:
            //Forward Difference
            for (i = istart; i <= iend; i++) {
                h[i] = 0;
                p = &kforw[1];
                for (a = -1; a <= 1; a++){
                    if(((i-a) > iend)||((i-a) < istart))
                        h[i] += 0;
                    else
                    h[i] += p[a]*f[i-a];
                }
            }
            strcpy(DerMthd, "Forward Difference");
            break;
        case 2:
            //Backward Difference
            for (i = istart; i <= iend; i++) {
                h[i] = 0;
                p = &kback[1];
                for (a = -1; a <= 1; a++){
                    if(((i-a) > iend)||((i-a) < istart))
                        h[i] += 0;
                    else
                    h[i] += p[a]*f[i-a];
                }
            }
            strcpy(DerMthd, "Backward Difference");
            break;
        case 3:
            //Central Difference
            for (i = istart; i <= iend; i++) {
                h[i] = 0;
                p = &kcentr[1];
                for (a = -1; a <= 1; a++){
                    if(((i-a) > iend)||((i-a) < istart))
                        h[i] += 0;
                    else
                    h[i] += p[a]*f[i-a];
                }
            }
            strcpy(DerMthd, "Central Difference");
            break;
        default:
            goto end;
            break;
    }
    printf("\n%s Derivative Computation result:\n\n"
    "h[n] = {%0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f}\n\n",DerMthd,h[0],h[1],h[2],h[3],h[4],h[5],h[6],h[7],h[8],h[9]);
    goto begin;
end:
    return 0;
}
