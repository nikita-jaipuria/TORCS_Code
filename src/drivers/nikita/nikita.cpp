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
#include <eigen3/Eigen/Dense>

#include <tgf.h> 
#include <track.h> 
#include <car.h> 
#include <raceman.h> 
#include <robottools.h>
#include <robot.h>

#include "driver.h"

#define BOTS 10

// static tTrack   *curTrack;

static const char* botname[BOTS] = {"nikita 1", "nikita 2", "nikita 3", "nikita 4", "nikita 5", "nikita 6", "nikita 7", "nikita 8", "nikita 9", "nikita 10"};
static const char* botdesc[BOTS] = {"nikita 1", "nikita 2", "nikita 3", "nikita 4", "nikita 5", "nikita 6", "nikita 7", "nikita 8", "nikita 9", "nikita 10"};

static Driver * driver[BOTS];
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
nikita(tModInfo *modInfo) 
{
    // printf("Module entry point achieved\n");
    memset(modInfo, 0, 10*sizeof(tModInfo));    
    for (int i = 0; i < BOTS; i++) {
        // std::cout << "YAYYY" << std::endl;
        modInfo[i].name    = strdup(botname[i]);        /* name of the module (short) */
        modInfo[i].desc    = strdup(botdesc[i]);  /* description of the module (can be long) */
        modInfo[i].fctInit = InitFuncPt;      /* init function */
        modInfo[i].gfId    = ROB_IDENT;       /* supported framework version */
        modInfo[i].index   = i+1;
    }
    // printf("Finished module entry point\n");
    return 0; 
} 

/* Module interface initialization. */
static int 
InitFuncPt(int index, void *pt) 
{ 
    tRobotItf *itf  = (tRobotItf *)pt; 
    // printf("module interface initialized for %d\n",index);    
    driver[index -1] = new Driver(index);
    itf->rbNewTrack = initTrack; /* Give the robot the track view called */ 
				 /* for every track change or new race */ 
    itf->rbNewRace  = newrace; 	 // Start a new race 
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
    
    driver[index-1]->initTrack(track, carHandle, carParmHandle, s);        
} 

/* Start a new race. */
static void  
newrace(int index, tCarElt* car, tSituation *s) 
{
    driver[index-1]->newRace(car, s);
} 

static void drive(int index, tCarElt* car, tSituation *s) 
{ 
    // std::cout << "drive" << std::endl;
    driver[index-1]->drive(car, s);   
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
     delete driver[index-1];             
}

