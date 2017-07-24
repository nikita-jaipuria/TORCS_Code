/***************************************************************************

    file                 : driver.h
    created              : Thu Apr 27 01:45 EST 2017
    copyright            : (C) 2002 Bernhard Wymann

 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#ifndef _DRIVER_H_
#define _DRIVER_H_

//include default header files
#include <boost/shared_ptr.hpp>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <stdlib.h> 
#include <string.h> 
#include <math.h>

//include the required header files, already defined within the TORCS source code
#include <tgf.h>
#include <track.h>
#include <car.h>
#include <raceman.h>
#include <robottools.h>
#include <robot.h>
#include "linalg.h"
#include "opponent.h"
class Opponents;
class Opponent;

class Driver {
    public:
        Driver(int index);
        ~Driver();
        /* callback functions called from TORCS */
        void initTrack(tTrack* t, void *carHandle, void **carParmHandle, tSituation *s);
        void newRace(tCarElt* car, tSituation *s);
        void drive(tCarElt* car, tSituation *s);
        void endRace(tCarElt *car, tSituation *s);
        tCarElt *getCarPtr() { return car; }
        tTrack *getTrackPtr() { return track; }
        float getSpeed() { return speed; }
    private:
        /* utility functions */
        bool isStuck(tCarElt* car);
        void update(tCarElt* car, tSituation *s);
        int getGear(tCarElt *car);
        float getBrake();
        void initCa();
        void initCw();
        float getEnvironmentPotential(tCarElt* car);
        float getCarPotential(tCarElt* car, tSituation* s);
        float getPotentialGradientY(tCarElt* car);
        float getPotentialGradientX(tCarElt* car);
        float getAllowedSpeed(tTrackSeg *segment);
        float getDistToSegEnd();
        float filterBColl(float brake); 

        Opponents *opponents;
        Opponent *opponent;

        /* per robot global data */
        int stuck;
        float trackangle;
        float angle;
        tCarElt *car;
        float mass;
        float CARMASS;
        float CA;
        float CW;
        float speed;    /* speed in track direction */
        float cur_y; // y-coordinate in the global Frenet FOR as defined in the artificial potential paper
        float cur_x;

        /* data that should stay constant after first initialization */
        int MAX_UNSTUCK_COUNT;
        int INDEX;
        float DESIRED_SPEED;

        /* class constants */
        static const float MAX_UNSTUCK_ANGLE;
        static const float UNSTUCK_TIME_LIMIT;
        static const float MAX_UNSTUCK_SPEED;
        static const float MIN_UNSTUCK_DIST;
        static const float SHIFT;
        static const float SHIFT_MARGIN;
        static const int nLanes;
        static const float deltaLane;
        static const float Alane;
        static const float _sigma;
        static const float _neta;
        static const float Acar;
        static const float _alpha;
        static const float delta_t;
        static const float _beta;
        static const float Tf;
        static const float _gamma; //tuned to work with TORCS
        static const float GAUSSIAN_DENOMINATOR; /* [m2] */

        /* track variables */
        tTrack* track;
};

#endif // _DRIVER_H_