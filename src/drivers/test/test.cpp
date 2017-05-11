/***************************************************************************

    file                 : test.cpp
    created              : Fri Apr 21 13:22:26 EDT 2017
    copyright            : (C) 2002 Nikita

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

#include <stdio.h>
#include <stdlib.h> 
#include <string.h> 
#include <math.h>

#include <tgf.h> 
#include <track.h> 
#include <car.h> 
#include <raceman.h> 
#include <robottools.h>
#include <robot.h>

#include "driver.h"

#define BOTS 3
#define BUFSIZE 20 //defines string length for botname[i]

// static tTrack    *curTrack;

static const char* botname[BOTS] = {"test 1", "test 2", "test 3"};
static const char* botdesc[BOTS] = {"test 1", "test 2", "test 3"};

static Driver* driver[BOTS];

static void initTrack(int index, tTrack* track, void *carHandle, void **carParmHandle, tSituation *s); 
static void newrace(int index, tCarElt* car, tSituation *s); 
static void drive(int index, tCarElt* car, tSituation *s); 
static int  pitcmd(int index, tCarElt* car, tSituation *s);
static void endrace(int index, tCarElt *car, tSituation *s);
static void shutdown(int index);
static int  InitFuncPt(int index, void *pt); 

/* 
 * Module entry point  
 */ 
extern "C" int 
test(tModInfo *modInfo) 
{
    char buffer[BUFSIZE];
    /* clear all structures */
    memset(modInfo, 0, 10*sizeof(tModInfo));

    for (int i = 0; i < BOTS; i++) {
        modInfo[i].name = strdup(botname[i]);           /* name of the module (short) */
        modInfo[i].desc = strdup(botdesc[i]);                   /* description of the module (can be long) */
        modInfo[i].fctInit = InitFuncPt;        /* init function */
        modInfo[i].gfId    = ROB_IDENT;         /* supported framework version */
        modInfo[i].index   = i+1;               /* has to be the same as the index in .xml */
        // printf("module entry point created for %d\n", modInfo[i].index);
    }
    return 0;
} 

/* Module interface initialization. */
static int 
InitFuncPt(int index, void *pt) 
{ 
    tRobotItf *itf  = (tRobotItf *)pt; 
    // printf("module interface initialized for %d\n",index);
    /* create robot instance for index */
    driver[index-1] = new Driver(index); /* calling function Driver */
    itf->rbNewTrack = initTrack; /* Give the robot the track view called */ 
                                 /* for every track change or new race */ 
    itf->rbNewRace  = newrace;   /* Start a new race */
    itf->rbDrive    = drive;     /* Drive during race */
    itf->rbPitCmd   = pitcmd;
    itf->rbEndRace  = endrace;   /* End of the current race */
    itf->rbShutdown = shutdown;  /* Called before the module is unloaded */
    itf->index      = index;     /* Index used if multiple interfaces */
    return 0; 
} 

/* Called for every track change or new race. */
static void  
initTrack(int index, tTrack* track, void *carHandle, void **carParmHandle, tSituation *s) 
{ 
    // printf("initializing the driver number %d\n", index);
    driver[index-1]->initTrack(track, carHandle, carParmHandle, s);
} 

/* Start a new race. */
static void  
newrace(int index, tCarElt* car, tSituation *s) 
{
    driver[index-1]->newRace(car, s); 
} 

/* Drive during race. */
static void  
drive(int index, tCarElt* car, tSituation *s) 
{ 
    driver[index-1]->drive(car, s);
    // float angle;
    // const float SC = 1.0;

    // memset((void *)&car->ctrl, 0, sizeof(tCarCtrl)); 
    
    // if (isStuck(car)) {
    //     angle = -RtTrackSideTgAngleL(&(car->_trkPos)) + car->_yaw;
    //     NORM_PI_PI(angle); // put the angle back in the range from -PI to PI
    //     car->ctrl.steer = angle / car->_steerLock;
    //     car->ctrl.gear = -1; // reverse gear
    //     car->ctrl.accelCmd = 0.3; // 30% accelerator pedal
    //     car->ctrl.brakeCmd = 0.0; // no brakes
    // } else {
    //     angle = RtTrackSideTgAngleL(&(car->_trkPos)) - car->_yaw;
    //     NORM_PI_PI(angle); // put the angle back in the range from -PI to PI
    //     angle -= SC*car->_trkPos.toMiddle/car->_trkPos.seg->width;
    //     car->ctrl.steer = angle / car->_steerLock;
    //     car->ctrl.gear = 1; // first gear
    //     car->ctrl.accelCmd = 0.3; // 30% accelerator pedal
    //     car->ctrl.brakeCmd = 0.0; // no brakes
    // }
}

/* Pitstop callback */
static int pitcmd(int index, tCarElt* car, tSituation *s)
{
    return driver[index-1]->pitCommand(car, s);
}

/* End of the current race */
static void
endrace(int index, tCarElt *car, tSituation *s)
{
    driver[index-1]->endRace(car, s);
}

/* Called before the module is unloaded */
static void
shutdown(int index)
{
    // printf("freeing botname %d\n", index);
    // free(botname[index-1]);
    delete driver[index-1];
}
