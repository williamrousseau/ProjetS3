/*
Projet S3 GRO
Class to control a PID
@author Jean-Samuel Lauzon
@version 1.0 23/04/2019
*/

#include "PID.h"

PID::PID(){
}


PID::PID(double kp1, double ki1, double kd1, double kp2, double ki2, double kd2){
    Kp_1 = kp1; Ki_1 = ki1; Kd_1 = kd1;
    Kp_2 = kp2; Ki_2 = ki2; Kd_2 = kd2;
    }

void PID::enable1(){

    // if function pointers are initiated
    if(measurementFunc_1!=nullptr && commandFunc_!=nullptr){
        enable_1 = true;
        measureTime_ = millis() + dtMs_;
        atGoal_1 = false;
        eIntegral_ = 0;
        ePrevious = 0;
    }
}
void PID::enable2(){

    // if function pointers are initiated
    if(measurementFunc_2!=nullptr && commandFunc_!=nullptr){
        enable_2 = true;
        measureTime_ = millis() + dtMs_;
        atGoal_2 = false;
        eIntegral_ = 0;
        ePrevious = 0;
    }   
}

void PID::disable1(){
    enable_1 = false;
}void PID::disable2(){
    enable_2 = false;
}


void PID::run(){
    // if enabled and time to run iteration
    command_temp = 0;
    if(millis() >= measureTime_ && enable_1){            //1

        //actualDt_ = millis() - measureTime_;
        measureTime_ = millis() + dtMs_;
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
            command_temp = computeCommand1(error1);
        //    commandFunc_1(computeCommand1(error1));
        }
       lastMeasureTime_ =  measureTime_;
    }
    if(millis() >= measureTime_ && enable_2){            //2

        //actualDt_ = millis() - measureTime_;
        measureTime_ = millis() + dtMs_;
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
            command_temp = command_temp + computeCommand2(error2);           //A REVOIR!
        //    commandFunc_2(computeCommand2(error2));
        }
       lastMeasureTime_ =  measureTime_;
    }
    commandFunc_(command_temp);
}

double PID::computeCommand1(double error1){
    double CMD;
    actualDt_ = millis() - lastMeasureTime_;
    eIntegral_ += error1;
    
    // Integral saturation
    if(eIntegral_ > eIntegralLim_){
        eIntegral_ = eIntegralLim_;
    }
    if(eIntegral_ < -eIntegralLim_){
        eIntegral_ = -eIntegralLim_;
    }

    CMD = Kp_1*error1 + Ki_1*eIntegral_*dt_ + Kd_1*(error1-ePrevious)/dt_;

    ePrevious = error1;
    return CMD;
}
double PID::computeCommand2(double error2){
    double CMD;
    actualDt_ = millis() - lastMeasureTime_;
    eIntegral_ += error2;
    
    // Integral saturation
    if(eIntegral_ > eIntegralLim_){
        eIntegral_ = eIntegralLim_;
    }
    if(eIntegral_ < -eIntegralLim_){
        eIntegral_ = -eIntegralLim_;
    }

    CMD = Kp_2*error2 + Ki_2*eIntegral_*dt_ + Kd_1*(error2-ePrevious)/dt_;

    ePrevious = error2;
    return CMD;
}