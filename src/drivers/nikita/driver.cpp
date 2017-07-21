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
#include <iostream>
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

Driver::Driver(int index)
{
    std::cout << "YAY" << std::endl;
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
    desired_speed = (101+INDEX-1)/3.6; /* [m/s] */
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
        car->ctrl.steer = angle / car->_steerLock;
        car->ctrl.gear = -1; // reverse gear
        car->ctrl.accelCmd = 0.3; // 30% accelerator pedal
        car->ctrl.brakeCmd = 0.0; // no brakes
    } 
    else {
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
        car->ctrl.steer = steerangle / car->_steerLock;
        car->ctrl.gear = getGear(car);
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
    angle = -trackangle + car->_yaw;
    NORM_PI_PI(angle);
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
    float cur_y = car->_trkPos.toMiddle + 5.7;
    float uLane1 = Alane*exp(-pow(cur_y - deltaLane/2.0*5.0, 2)/GAUSSIAN_DENOMINATOR);
    float uLane2 = Alane*exp(-pow(cur_y - deltaLane/2.0*3.0, 2)/GAUSSIAN_DENOMINATOR);
    float uLane3 = Alane*exp(-pow(cur_y - deltaLane/2.0, 2)/GAUSSIAN_DENOMINATOR);
    float uRoad1 = 0.5*_neta*pow(1.0/(cur_y - (deltaLane/2.0*7.0 + 0.5)), 2);
    float uRoad2 = 0.5*_neta*pow(1.0/(cur_y - (-deltaLane -0.5)), 2);
    return uRoad2 + uRoad1 + uLane3 + uLane2 + uLane1;
}

// float Driver::getCarPotential(tCarElt* car, tSituation* s)
// {
//     float cur_x = ;
//     float cur_y = ;
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
    float cur_y = car->_trkPos.toMiddle + 5.7;
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
    float uVelGradX = _gamma*(car->pub.speed - desired_speed);
    float uCarGradX = 0.0; // check behaviour without obstacles
    return uVelGradX + uCarGradX;
}