/***************************************************************************

    file                 : chenyi_AI3.cpp
    created              : 2014年 09月 01日 星期一 13:08:25 EDT
    copyright            : (C) 2002 Chenyi Chen

 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#ifdef _WIN32
#include <windows.h>
#endif
#include <boost/shared_ptr.hpp>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <stdlib.h> 
#include <string.h> 
#include <math.h>

#include <tgf.h> 
#include <track.h> 
#include <car.h> 
#include <raceman.h> 
#include <robottools.h>
#include <robot.h>

static tTrack	*curTrack;

static void initTrack(int index, tTrack* track, void *carHandle, void **carParmHandle, tSituation *s); 
static void newrace(int index, tCarElt* car, tSituation *s); 
static void drive(int index, tCarElt* car, tSituation *s); 
static void endrace(int index, tCarElt *car, tSituation *s);
static void shutdown(int index);
static int  InitFuncPt(int index, void *pt); 


/* 
 * Module entry point  
 */ 
extern "C" int 
chenyi_AI3(tModInfo *modInfo) 
{
    memset(modInfo, 0, 10*sizeof(tModInfo));

    modInfo->name    = strdup("chenyi_AI3");		/* name of the module (short) */
    modInfo->desc    = strdup("");	/* description of the module (can be long) */
    modInfo->fctInit = InitFuncPt;		/* init function */
    modInfo->gfId    = ROB_IDENT;		/* supported framework version */
    modInfo->index   = 1;

    return 0; 
} 

/* Module interface initialization. */
static int 
InitFuncPt(int index, void *pt) 
{ 
    tRobotItf *itf  = (tRobotItf *)pt; 

    itf->rbNewTrack = initTrack; /* Give the robot the track view called */ 
				 /* for every track change or new race */ 
    itf->rbNewRace  = newrace; 	 /* Start a new race */
    itf->rbDrive    = drive;	 /* Drive during race */
    itf->rbPitCmd   = NULL;
    itf->rbEndRace  = endrace;	 /* End of the current race */
    itf->rbShutdown = shutdown;	 /* Called before the module is unloaded */
    itf->index      = index; 	 /* Index used if multiple interfaces */
    return 0; 
} 

/* Called for every track change or new race. */
static void  
initTrack(int index, tTrack* track, void *carHandle, void **carParmHandle, tSituation *s) 
{ 
    curTrack = track;
    *carParmHandle = NULL; 
} 

/* Start a new race. */
static void  
newrace(int index, tCarElt* car, tSituation *s) 
{ 
} 


/* Compute gear. */
const float SHIFT = 0.85;         /* [-] (% of rpmredline) */
const float SHIFT_MARGIN = 4.0;  /* [m/s] */

int getGear(tCarElt *car)
{
	if (car->_gear <= 0)
		return 1;
	float gr_up = car->_gearRatio[car->_gear + car->_gearOffset];
	float omega = car->_enginerpmRedLine/gr_up;
	float wr = car->_wheelRadius(2);

	if (omega*wr*SHIFT < car->_speed_x) {
		return car->_gear + 1;
	} else {
		float gr_down = car->_gearRatio[car->_gear + car->_gearOffset - 1];
		omega = car->_enginerpmRedLine/gr_down;
		if (car->_gear > 1 && omega*wr*SHIFT > car->_speed_x + SHIFT_MARGIN) {
			return car->_gear - 1;
		}
	}
	return car->_gear;
}


/* check if the car is stuck */
const float MAX_UNSTUCK_SPEED = 5.0;   /* [m/s] */
const float MIN_UNSTUCK_DIST = 3.0;    /* [m] */
const float MAX_UNSTUCK_ANGLE = 20.0/180.0*PI;
const int MAX_UNSTUCK_COUNT = 250;
static int stuck = 0;

bool isStuck(tCarElt* car)
{
    float angle = RtTrackSideTgAngleL(&(car->_trkPos)) - car->_yaw;
    NORM_PI_PI(angle);

    if (fabs(angle) > MAX_UNSTUCK_ANGLE &&
        car->_speed_x < MAX_UNSTUCK_SPEED &&
        fabs(car->_trkPos.toMiddle) > MIN_UNSTUCK_DIST) {
        if (stuck > MAX_UNSTUCK_COUNT && car->_trkPos.toMiddle*angle < 0.0) {
            return true;
        } else {
            stuck++;
            return false;
        }
    } else {
        stuck = 0;
        return false;
    }
}

const int nLanes = 4;
const float deltaLane = 3.8;
// const float W = ;
// const float L = ;
const float Alane = 2;
const float _sigma = 0.3*deltaLane;
const float _neta = 3;
const float Acar = 10;
const float _alpha = 0.5;
const float delta_t = -0.5;
const float _beta = 0.6;
const float Tf = 3;
const float _gamma = 0.2; //tuned to work with TORCS
const float GAUSSIAN_DENOMINATOR = 2.0*_sigma*_sigma; /* [m2] */

// for indexing of road and lane edges, followed the same directional convention as for lanes
float getEnvironmentPotential(tCarElt* car)
{
    float cur_y = car->_trkPos.toMiddle + 5.7;
    float uLane1 = Alane*exp(-pow(cur_y - deltaLane/2.0*5.0, 2)/GAUSSIAN_DENOMINATOR);
    float uLane2 = Alane*exp(-pow(cur_y - deltaLane/2.0*3.0, 2)/GAUSSIAN_DENOMINATOR);
    float uLane3 = Alane*exp(-pow(cur_y - deltaLane/2.0, 2)/GAUSSIAN_DENOMINATOR);
    float uRoad1 = 0.5*_neta*pow(1.0/(cur_y - (deltaLane/2.0*7.0 + 0.5)), 2);
    float uRoad2 = 0.5*_neta*pow(1.0/(cur_y - (-deltaLane -0.5)), 2);
    return uRoad2 + uRoad1 + uLane3 + uLane2 + uLane1;
}

// float getCarVelocityPotentials(tCarElt* car)
// {
//     float cur_x = ;
//     float uVel = _gamma*(car->pub.speed - desired_speed)*cur_x;
//     float uCar = ; 
//     int m = ; // number of relevant obstacle cars
//     for (int i = 0; i < m; i++) {
//         float K = ; // pseudo-distance to m-th obstacle car
//         uCar += Acar*exp( - _alpha*K)/K;
//     }
//     return UVel + uCar;
// }

float getPotentialGradientY(tCarElt* car)
{
    float cur_y = car->_trkPos.toMiddle + 5.7;
    float uLane1GradY = Alane*exp(-pow(cur_y - deltaLane/2.0*5.0, 2)/GAUSSIAN_DENOMINATOR)*(-2*(cur_y - deltaLane/2.0*5.0)/GAUSSIAN_DENOMINATOR);
    float uLane2GradY = Alane*exp(-pow(cur_y - deltaLane/2.0*3.0, 2)/GAUSSIAN_DENOMINATOR)*(-2*(cur_y - deltaLane/2.0*3.0)/GAUSSIAN_DENOMINATOR);
    float uLane3GradY = Alane*exp(-pow(cur_y - deltaLane/2.0, 2)/GAUSSIAN_DENOMINATOR)*(-2*(cur_y - deltaLane/2.0)/GAUSSIAN_DENOMINATOR);
    float uRoad1GradY = 0.5*_neta*(-2)*pow(1.0/(cur_y - (deltaLane/2.0*7.0 + 0.5)), 3);
    float uRoad2GradY = 0.5*_neta*(-2)*pow(1.0/(cur_y - (-deltaLane -0.5)), 3);
    float uCarGradY = 0.0; // check behaviour without obstacles
    return uLane1GradY + uLane2GradY + uLane3GradY + uRoad1GradY + uRoad2GradY + uCarGradY;
}

float getPotentialGradientX(tCarElt* car, double speed)
{
    float uVelGradX = _gamma*(car->pub.speed - speed);
    float uCarGradX = 0.0; // check behaviour without obstacles
    return uVelGradX + uCarGradX;
}

/* Drive during race. */
boost::shared_ptr<std::ofstream> myfile;
const double desired_speed=101/3.6;
//double keepLR=1.9;// for three-lane

static void drive(int index, tCarElt* car, tSituation *s) 
{ 
    memset(&car->ctrl, 0, sizeof(tCarCtrl));
    if (!myfile) {
        myfile.reset(new std::ofstream());
        myfile->open ("sensor_readings.txt","w+");
    }
    if (isStuck(car)) {
        float angle = -RtTrackSideTgAngleL(&(car->_trkPos)) + car->_yaw;
        NORM_PI_PI(angle); // put the angle back in the range from -PI to PI

        car->ctrl.steer = angle / car->_steerLock;
        car->ctrl.gear = -1; // reverse gear
        car->ctrl.accelCmd = 0.3; // 30% accelerator pedal
        car->ctrl.brakeCmd = 0.0; // no brakes
    } 
    else {
        float angle;
        const float SC = 1.0;

        angle = RtTrackSideTgAngleL(&(car->_trkPos)) - car->_yaw;
        NORM_PI_PI(angle); // put the angle back in the range from -PI to PI

        float fx = -getPotentialGradientX(car, desired_speed);
        float fy = -getPotentialGradientY(car);

        // Transform force obtained in Frenet coordinates into the car axis FOR for control
        float gain = 3.5;
        float accel_x = gain*(fx*cos(angle) - fy*sin(angle));
        float accel_y = gain*(fx*sin(angle) + fy*cos(angle));
        float accel_mag = sqrt(pow(accel_x,2) + pow(accel_y,2));
        float desired_angle = atan(accel_y/accel_x);
        NORM_PI_PI(desired_angle); // put the angle back in the range from -PI to PI

        std::ofstream myfile;
        
        myfile << accel_x << "," << accel_y << "," << desired_angle << "," << car->_trkPos.toMiddle << std::endl;
    
        
        car->ctrl.steer = desired_angle / car->_steerLock;
        car->ctrl.gear = getGear(car);

        if (-desired_angle > -PI/2.0  && -desired_angle < PI/2.0) {
            if (accel_mag/15.0 < 1.0) {
                car->ctrl.accelCmd = accel_mag/15.0;
            }
            else {
                car->ctrl.accelCmd = 1.0;
            }
        }
        else {
            if (accel_mag/15.0 < 1.0) {
                car->ctrl.brakeCmd = accel_mag/15.0;
            }
            else {
                car->ctrl.brakeCmd = 1.0;
            }
        }

        // // set up the values to return
        // car->ctrl.steer = angle / car->_steerLock;
        // car->ctrl.gear = getGear(car);

        // if (car->_speed_x>desired_speed) {
        //    car->ctrl.brakeCmd=0.5;
        //    car->ctrl.accelCmd=0.0;
        // }
        // else if  (car->_speed_x<desired_speed) {
        //    car->ctrl.accelCmd=0.5;
        //    car->ctrl.brakeCmd=0.0;
        // }
    }    
}

/* End of the current race */
static void
endrace(int index, tCarElt *car, tSituation *s)
{
}

/* Called before the module is unloaded */
static void
shutdown(int index)
{
    if (myfile) myfile->close();
}

