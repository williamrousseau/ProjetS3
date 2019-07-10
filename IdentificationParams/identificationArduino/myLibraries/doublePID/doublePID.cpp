/*
Projet S3 GRO
Class to control a PID
@author Jean-Samuel Lauzon
@version 1.0 23/04/2019
*/

#include "doublePID.h"

doublePID::doublePID(){
}


doublePID::doublePID(double kp1, double ki1, double kd1, double kp2, double ki2, double kd2){
    Kp_1 = kp1; Ki_1 = ki1; Kd_1 = kd1;
    Kp_2 = kp2; Ki_2 = ki2; Kd_2 = kd2;
    }

void doublePID::enable(){

    // if function pointers are initiated
    if(measurementFunc_1!=nullptr && commandFunc_!=nullptr){
        enable_1 = true;
        enable_2 = true;
        measureTime_ = millis() + dtMs_;
        atGoal_1 = false;
        atGoal_2 = false;
        eIntegral_1 = 0;
        eIntegral_2 = 0;
        ePrevious1 = 0;
        ePrevious2 = 0;
    }
}

void doublePID::setWeight(double wPID1, double wPID2){
    weightPID_1 = wPID1; weightPID_2 = wPID2;
}

void doublePID::disable(){
    enable_1 = false;
    enable_2 = false;
}


void doublePID::run(){
    // if enabled and time to run iteration
    if(millis() >= measureTime_){
        measureTime_ = millis() + dtMs_;
        commandPID_1 = 0;
        commandPID_2 = 0;
        if(enable_1){                                           //1
            double error1 = goal_1 - measurementFunc_1();
            
            // if goal reached
            if(fabs(error1)<epsilon_1){

                atGoal_1 = true;
                enable_1 = false;
                if (atGoalFunc1 != nullptr){
                    atGoalFunc1();
                }
            }
            else{
                commandPID_1 = computeCommand1(error1);
            }
        }
        if(enable_2){                                            //2
            double error2 = goal_2 - measurementFunc_2();
            // if goal reached
            if(fabs(error2)<epsilon_2){
                atGoal_2 = true;
                enable_2 = false;
                if (atGoalFunc2 != nullptr){
                    atGoalFunc2();
                }
            }
            else{
                commandPID_2 = computeCommand2(error2);
            }
        }
        lastMeasureTime_ =  measureTime_;
        commandFunc_(weightPID_1 * commandPID_1 + weightPID_2 * commandPID_2);
    }
    
}

double doublePID::computeCommand1(double error1){
    double CMD;
    actualDt_ = millis() - lastMeasureTime_;
    eIntegral_1 += error1;
    
    // Integral saturation
    if(eIntegral_1 > eIntegralLim_1){
        eIntegral_1 = eIntegralLim_1;
    }
    if(eIntegral_1 < -eIntegralLim_1){
        eIntegral_1 = -eIntegralLim_1;
    }

    CMD = Kp_1*error1 + Ki_1*eIntegral_1*dt_ + Kd_1*(error1-ePrevious1)/dt_;

    ePrevious1 = error1;
    return CMD;
}
double doublePID::computeCommand2(double error2){
    double CMD;
    actualDt_ = millis() - lastMeasureTime_;
    eIntegral_2 += error2;
    
    // Integral saturation
    if(eIntegral_2 > eIntegralLim_2){
        eIntegral_2 = eIntegralLim_2;
    }
    if(eIntegral_2 < -eIntegralLim_2){
        eIntegral_2 = -eIntegralLim_2;
    }

    CMD = Kp_2*error2 + Ki_2*eIntegral_2*dt_ + Kd_1*(error2-ePrevious2)/dt_;

    ePrevious2 = error2;
    return CMD;
}