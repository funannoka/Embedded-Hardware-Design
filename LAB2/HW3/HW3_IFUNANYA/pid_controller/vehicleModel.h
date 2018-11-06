//
//  vehicleModel.h
//  pid_controller
//
//  Created by Ifunanya Nnoka on 4/9/17.
//  Copyright © 2017 Ifunanya Nnoka. All rights reserved.
//

#ifndef vehicleModel_h
#define vehicleModel_h


#define SPDVHL      1.389     // vehicle speed 5 KM per hour
#define STPFUL      1.8       //Full step
#define STPHAL      0.9       //Half step
#define STPQTR      0.45       //Quarter step
#define STPCON      0.125    // 1/8
#define STPMOT      0.225     // 1/8 micro step of the steering motor
#define PWMMOT      500       // pwm max frequency to drive the motor controller 500 hz
#define PMWTIM      0.002         //1/PMWMOT  //Time interval of each pmw pulse
#define ANGSTR      M_PI/3      // max steering angle
#define RADWHL      0.3       // radius of the wheel
#define ANGSMP      30        // angle sampling rate from i2c sensor
#define DSTMIN      26.51          //PMWMOT*STPCON*2*M_PI*RADWHL*STPMOT
#define DSTMAX      212.08          //PMWMOT*STPCON*2*M_PI*RADWHL*STPMOT

#endif /* vehicleModel_h */
