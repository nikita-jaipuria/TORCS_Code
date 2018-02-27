/***************************************************************************

    file                 : driver.cpp
    created              : Thu Apr 27 01:56 EST 2017
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

#include "driver.h"
const float Driver::SHIFT = 0.85;         /* [-] (% of rpmredline) */
const float Driver::SHIFT_MARGIN = 4.0;  /* [m/s] */
const float Driver::MAX_UNSTUCK_ANGLE = 30.0/180.0*PI;  /* [radians] */
const float Driver::UNSTUCK_TIME_LIMIT = 2.0;           /* [s] */
const float Driver::MAX_UNSTUCK_SPEED = 5.0;   /* [m/s] */
const float Driver::MIN_UNSTUCK_DIST = 3.0;    /* [m] */
const int Driver::nLanes = 4;
const float Driver::deltaLane = 3.8;
const float Driver::Alane = 2;
const float Driver::_sigma = 0.3*deltaLane;
const float Driver::_neta = 3;
const float Driver::Acar = 10;
const float Driver::_alpha = 0.5;
const float Driver::delta_t = -0.5;
const float Driver::_beta = 0.6;
const float Driver::Tf = 3;
const float Driver::_gamma = 0.2; //tuned to work with TORCS
const float Driver::GAUSSIAN_DENOMINATOR = 2.0*_sigma*_sigma; /* [m2] */
const float Driver::SIDECOLL_MARGIN = 2.0;   /* [m] */

Driver::Driver(int index)
{
    // std::cout << "YAY" << std::endl;
    INDEX = index;
}

/* Called for every track change or new race. */
void Driver::initTrack(tTrack* t, void *carHandle, void **carParmHandle, tSituation *s)
{
    track = t;
    *carParmHandle = NULL;
}

/* Start a new race. */
void Driver::newRace(tCarElt* car, tSituation *s)
{
    MAX_UNSTUCK_COUNT = int(UNSTUCK_TIME_LIMIT/RCM_MAX_DT_ROBOTS);
    stuck = 0;
    this->car = car;
    CARMASS = GfParmGetNum(car->_carHandle, SECT_CAR, PRM_MASS, NULL, 1000.0);
    initCa();
    initCw();
    mass = CARMASS + car->_fuel;
    DESIRED_SPEED = (100+INDEX*5-5)/3.6; /* [m/s] */
    std::cout << DESIRED_SPEED << std::endl;
    /* initialize the list of opponents */
    opponents = new Opponents(s, this);
    opponent = opponents->getOpponentPtr();
}

/* Drive during race. */
void Driver::drive(tCarElt* car, tSituation* s)
{
    update(car, s);

    memset(&car->ctrl, 0, sizeof(tCarCtrl));
    // if (!myfile) {
    //     myfile.reset(new std::ofstream());
    //     myfile->open ("sensor_readings.txt","w+");
    // }

    if (isStuck(car)) {
        // std::cout << "OOPS" << INDEX << std::endl;
        car->ctrl.steer = angle / car->_steerLock;
        car->ctrl.gear = -1; // reverse gear
        car->ctrl.accelCmd = 0.3; // 30% accelerator pedal
        car->ctrl.brakeCmd = 0.0; // no brakes
    } 
    else {
        // std::cout << "YAY" << INDEX << std::endl;
        float fx = -getPotentialGradientX(car);
        float fy = -getPotentialGradientY(car);

        // Transform force obtained in Frenet coordinates into the car axis FOR for control
        float gain = 2.0;
        float accel_x = gain*(fx*cos(angle) - fy*sin(angle));
        float accel_y = gain*(fx*sin(angle) + fy*cos(angle));
        float accel_mag = sqrt(pow(accel_x,2) + pow(accel_y,2));
        float steerangle = atan(accel_y/accel_x);
        NORM_PI_PI(steerangle); // put the angle back in the range from -PI to PI
        // std::ofstream myfile;        
        // myfile << accel_x << "," << accel_y << "," << desired_angle << "," << car->_trkPos.toMiddle << std::endl;           
        car->ctrl.steer = filterSColl(steerangle / car->_steerLock);
        car->ctrl.gear = getGear(car);
        car->ctrl.brakeCmd = filterBColl(getBrake());
        float accel_mag_norm = accel_mag/15.0; //to make car achieve desired speed
        if (steerangle > -PI/2.0  && steerangle < PI/2.0) {
            if (accel_mag_norm < 1.0) {
                car->ctrl.accelCmd = accel_mag_norm;
            }
            else {
                car->ctrl.accelCmd = 1.0;
            }
        }
        else {
            if (accel_mag_norm < 1.0) {
                car->ctrl.brakeCmd = accel_mag_norm;
            }
            else {
                car->ctrl.brakeCmd = 1.0;
            }
        }
    } 
}

/* End of the current race */
void Driver::endRace(tCarElt *car, tSituation *s)
{
}

/* Update my private data every timestep */
void Driver::update(tCarElt* car, tSituation *s)
{
    trackangle = RtTrackSideTgAngleL(&(car->_trkPos));
    angle = trackangle - car->_yaw;
    NORM_PI_PI(angle);
    cur_y = car->_trkPos.toMiddle + 5.7;
    // cur_x = 
    speed = Opponent::getSpeed(car);
    opponents->update(s, this);
}

/* Check if I'm stuck */
bool Driver::isStuck(tCarElt* car)
{
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

/* Compute gear */
int Driver::getGear(tCarElt* car)
{
    if (car->_gear <= 0) return 1;
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

// for indexing of road and lane edges, followed the same directional convention as for lanes
float Driver::getEnvironmentPotential(tCarElt* car)
{
    float uLane1 = Alane*exp(-pow(cur_y - deltaLane/2.0*5.0, 2)/GAUSSIAN_DENOMINATOR);
    float uLane2 = Alane*exp(-pow(cur_y - deltaLane/2.0*3.0, 2)/GAUSSIAN_DENOMINATOR);
    float uLane3 = Alane*exp(-pow(cur_y - deltaLane/2.0, 2)/GAUSSIAN_DENOMINATOR);
    float uRoad1 = 0.5*_neta*pow(1.0/(cur_y - (deltaLane/2.0*7.0 + 0.5)), 2);
    float uRoad2 = 0.5*_neta*pow(1.0/(cur_y - (-deltaLane -0.5)), 2);
    return uRoad2 + uRoad1 + uLane3 + uLane2 + uLane1;
}

// float Driver::getCarPotential(tCarElt* car, tSituation* s)
// {
//     float uCar = ; 
//     int m = ; // number of relevant obstacle cars
//     for (int i = 0; i < m; i++) {
//         float K = ; // pseudo-distance to m-th obstacle car
//         uCar += Acar*exp( - _alpha*K)/K;
//     }
//     return UVel + uCar;
// }

float Driver::getPotentialGradientY(tCarElt* car)
{
    float uLane1GradY = Alane*exp(-pow(cur_y - deltaLane/2.0*5.0, 2)/GAUSSIAN_DENOMINATOR)*(-2*(cur_y - deltaLane/2.0*5.0)/GAUSSIAN_DENOMINATOR);
    float uLane2GradY = Alane*exp(-pow(cur_y - deltaLane/2.0*3.0, 2)/GAUSSIAN_DENOMINATOR)*(-2*(cur_y - deltaLane/2.0*3.0)/GAUSSIAN_DENOMINATOR);
    float uLane3GradY = Alane*exp(-pow(cur_y - deltaLane/2.0, 2)/GAUSSIAN_DENOMINATOR)*(-2*(cur_y - deltaLane/2.0)/GAUSSIAN_DENOMINATOR);
    float uRoad1GradY = 0.5*_neta*(-2)*pow(1.0/(cur_y - (deltaLane/2.0*7.0 + 0.5)), 3);
    float uRoad2GradY = 0.5*_neta*(-2)*pow(1.0/(cur_y - (-deltaLane -0.5)), 3);
    float uCarGradY = 0.0; // check behaviour without obstacles
    return uLane1GradY + uLane2GradY + uLane3GradY + uRoad1GradY + uRoad2GradY + uCarGradY;
}

float Driver::getPotentialGradientX(tCarElt* car)
{
    float uVelGradX = _gamma*(speed - DESIRED_SPEED);
    float uCarGradX = 0.0; // check behaviour without obstacles
    return uVelGradX + uCarGradX;
}

/* Brake filter for collision avoidance */
float Driver::filterBColl(float brake)
{
    float currentspeedsqr = car->_speed_x*car->_speed_x;
    float mu = car->_trkPos.seg->surface->kFriction;
    float cm = mu*G*mass;
    float ca = CA*mu + CW;
    int i;

    for (i = 0; i < opponents->getNOpponents(); i++) {
        if (opponent[i].getState() & OPP_COLL) {
            float allowedspeedsqr = opponent[i].getSpeed();
            allowedspeedsqr *= allowedspeedsqr;
            float brakedist = mass*(currentspeedsqr - allowedspeedsqr) /
                              (2.0*(cm + allowedspeedsqr*ca));
            if (brakedist > opponent[i].getDistance()) {
                return 1.0;
            }
        }
    }
    return brake;
}

/* Steer filter for collision avoidance */
float Driver::filterSColl(float steer)
{
    int i;
    float sidedist = 0.0, fsidedist = 0.0, minsidedist = FLT_MAX;
    Opponent *o = NULL;

    /* get the index of the nearest car (o) */
    for (i = 0; i < opponents->getNOpponents(); i++) {
        if (opponent[i].getState() & OPP_SIDE) {
            sidedist = opponent[i].getSideDist();
            fsidedist = fabs(sidedist);
            if (fsidedist < minsidedist) {
                minsidedist = fsidedist;
                o = &opponent[i];
            }
        }
    }
    /* if there is another car handle the situation */
    if (o != NULL) {
        float d = fsidedist - o->getWidth();
        /* near enough */
        if (d < SIDECOLL_MARGIN) {
            /* compute angle between cars */
            tCarElt *ocar = o->getCarPtr();
            float diffangle = ocar->_yaw - car->_yaw;
            NORM_PI_PI(diffangle);
            const float c = SIDECOLL_MARGIN/2.0;
            d = d - c;
            if (d < 0.0) d = 0.0;
            float psteer = diffangle/car->_steerLock;
            return steer*(d/c) + 2.0*psteer*(1.0-d/c);
        }
    }
    return steer;
}

float Driver::getBrake()
{
    tTrackSeg *segptr = car->_trkPos.seg;
    float currentspeedsqr = car->_speed_x*car->_speed_x;
    float mu = segptr->surface->kFriction;
    float maxlookaheaddist = currentspeedsqr/(2.0*mu*G);
    float lookaheaddist = getDistToSegEnd();
    float allowedspeed = getAllowedSpeed(segptr);
    if (allowedspeed < car->_speed_x) return 1.0;
    segptr = segptr->next;
    while (lookaheaddist < maxlookaheaddist) {
        allowedspeed = getAllowedSpeed(segptr);
        if (allowedspeed < car->_speed_x) {
            float allowedspeedsqr = allowedspeed*allowedspeed;
            float brakedist = mass*(currentspeedsqr - allowedspeedsqr) / (2.0*(mu*G*mass + allowedspeedsqr*(CA*mu + CW)));
            if (brakedist > lookaheaddist) {
                return 1.0;
            }
        }
        lookaheaddist += segptr->length;
        segptr = segptr->next;
    }
    return 0.0;
}

/* Compute aerodynamic downforce coefficient CA */
void Driver::initCa()
{
    char *WheelSect[4] = {SECT_FRNTRGTWHEEL, SECT_FRNTLFTWHEEL, SECT_REARRGTWHEEL, SECT_REARLFTWHEEL};
    float rearwingarea = GfParmGetNum(car->_carHandle, SECT_REARWING, PRM_WINGAREA, (char*) NULL, 0.0);
    float rearwingangle = GfParmGetNum(car->_carHandle, SECT_REARWING, PRM_WINGANGLE, (char*) NULL, 0.0);
    float wingca = 1.23*rearwingarea*sin(rearwingangle);
    float cl = GfParmGetNum(car->_carHandle, SECT_AERODYNAMICS, PRM_FCL, (char*) NULL, 0.0) + GfParmGetNum(car->_carHandle, SECT_AERODYNAMICS, PRM_RCL, (char*) NULL, 0.0);
    float h = 0.0;
    int i;
    for (i = 0; i < 4; i++)
        h += GfParmGetNum(car->_carHandle, WheelSect[i], PRM_RIDEHEIGHT, (char*) NULL, 0.20);
        h*= 1.5; h = h*h; h = h*h; h = 2.0 * exp(-3.0*h);
        CA = h*cl + 4.0*wingca;
}

/* Compute aerodynamic drag coefficient CW */
void Driver::initCw()
{
    float cx = GfParmGetNum(car->_carHandle, SECT_AERODYNAMICS,
                            PRM_CX, (char*) NULL, 0.0);
    float frontarea = GfParmGetNum(car->_carHandle, SECT_AERODYNAMICS,
                                   PRM_FRNTAREA, (char*) NULL, 0.0);
    CW = 0.645*cx*frontarea;
}

/* Compute the allowed speed on a segment */
float Driver::getAllowedSpeed(tTrackSeg *segment)
{
    if (segment->type == TR_STR) {
        return FLT_MAX;
    } else {
        float arc = 0.0;
        tTrackSeg *s = segment;
        
        while (s->type == segment->type && arc < PI/2.0) {
            arc += s->arc;
            s = s->next;
        }
        arc /= PI/2.0;
        float mu = segment->surface->kFriction;
        float r = (segment->radius + segment->width/2.0)/sqrt(arc);
        return sqrt((mu*G*r)/(1.0 - MIN(1.0, r*CA*mu/mass)));
    }
}

/* Compute the length to the end of the segment */
float Driver::getDistToSegEnd()
{
    if (car->_trkPos.seg->type == TR_STR) {
        return car->_trkPos.seg->length - car->_trkPos.toStart;
    } else {
        return (car->_trkPos.seg->arc - car->_trkPos.toStart)*car->_trkPos.seg->radius;
    }
}

Driver::~Driver()
{
    delete opponents;
}
