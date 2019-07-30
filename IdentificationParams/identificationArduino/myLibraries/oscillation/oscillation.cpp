/*
Projet S3 GRO
Class to control a PID
@author Emiles
@version 1.0 23/04/2019
*/

#include <Arduino.h>
#include "oscillation.h"
#define POTPIN          A5  

oscillation::oscillation()
{
    pointeActivite = 0;        //PARAMS
    Accel = 0;                   //À
    rapportSafety = 0.7;
    angleMin = 0;
    maxPos = 0;            
    noSlipCommand = 0;         //CHANGER
    tailleAngle = 0;
    capaciteAngle = 10;
    omega = 0;
    dtMs_ = 1; //période de 1 ms
    enabled = 0;
    lastCommand = 0;
    j = 0;
    anglePrec = 0;
    sens = 0;
    topAngle = 0;
    belowAngle = 0;
    
    angleMax = 0;
    
    
}

void oscillation::enable()
{
    measureTime_[0] = millis() + dtMs_;
    enabled = 1;
    startup = 1;
    for (int i=0; i<capaciteAngle; i++)
    {
        tableauAngle[i] = 0;
    }
}

void oscillation::disable()
{
    measureTime_[0] = 0;
    enabled = 0;
    startup = 1;
    tailleAngle = 0;
    omega = 0;  
    lastCommand = 0;
    j = 0;
    anglePrec = 0;
    sens = 0;
    topAngle = 0;
    belowAngle = 0;
    angleMax = 0;    
}

void oscillation::run()
{   
    if (millis() >= measureTime_[0])
    {
        measureTime_[0] = millis() + dtMs_;
        if (enabled)
        {
            commandeOscillation(measurementFunc1(), Accel);
        }
    }
}

void oscillation::commandeOscillation(double angle, float Acceleration)
{
    float commande = 0;
    vitesseAngulaire(angle);
    if (startup)
    {   
        //Serial.println("OOOOOOOOOOOOOOOO");
        if (startup == 1)
        {
            commande = ((rapportSafety*0.25*maxPos)-abs((rapportSafety*0.25*maxPos)-measurementFunc2()))/(rapportSafety*0.5*maxPos);
            if (commande < noSlipCommand) commande = noSlipCommand;
            if(measurementFunc2()>=0.7*rapportSafety*maxPos)
            {
                commande = 0;
                if (omega > 0)
                {
                    startup = 2;
                }
            }
        }
        if (startup == 2)
        {
            commande = -((rapportSafety*0.5*maxPos)-abs((rapportSafety*0.5*maxPos)-measurementFunc2()))/(rapportSafety*0.5*maxPos);
            if (commande > -noSlipCommand) commande = -noSlipCommand;
            if(measurementFunc2()<rapportSafety*0.5*maxPos)
            {
                commande = 0;
                startup = 0;
                if(omega<0) sens = -1;
                if(omega>0) sens = 1;
            }
        }

        commandFunc(commande);
    }
    if (omega < 0 && startup == 0)
    {
        //Serial.println("-----------");
        if(sens == -1)
        {   
            angleMax = angle;
            topAngle = abs(angle)*pointeActivite;
            belowAngle = -abs(angle)*pointeActivite;
            if (topAngle < angleMin) topAngle = angleMax;
            if (belowAngle > -angleMin) belowAngle = -angleMax;
        }
        sens = 1;
        if(angle < topAngle && angle > belowAngle)
        {   
            if (measurementFunc2() > maxPos*rapportSafety)
            {
                commande = 0;
            }
            else if (angle >= 0)
            {
                commande = Acceleration*(topAngle-angle)/topAngle;
            }
            else if (angle < 0)
            {
                commande = Acceleration*(belowAngle-angle)/belowAngle;
            }
            if (commande>1)
            {
                commande = 1;
            }
            commandFunc(commande);
        }
        else
        {
            commandFunc(0);
        }
    }
    else if (omega > 0 && startup == 0)
    {
        //Serial.println("-----------");
        if(sens == 1)
        {   
            topAngle = angle*pointeActivite;
            belowAngle = -angle*pointeActivite;
            if (topAngle > -angleMin) topAngle = -angleMax;
            if (belowAngle < angleMin) belowAngle = angleMax;
        }
        sens = -1;
        if(angle > topAngle && angle < belowAngle)
        {
            if (measurementFunc2() < 0.05)
            {
                commande = 0;
            }
            else if (angle <= 0)
            {
                commande = -Acceleration*(topAngle-angle)/topAngle;
            }
            else if (angle > 0)
            {
                commande = -Acceleration*(belowAngle-angle)/belowAngle;
            }
            if (commande<-1)
            {
                commande = -1;
            }
            commandFunc(commande);
        }
        else
        {
            commandFunc(0);
        }
    }
    else if (startup == 0)
    {
        //Serial.println("-----------");
        commandFunc(commande);
    }
    
}

void oscillation::vitesseAngulaire(double angle)
{
    float angleLive = 0;
    tableauAngle[tailleAngle] = angle;
    tailleAngle++;
    float somme = 0;

    if (tailleAngle == capaciteAngle)
    {
        for (int i=0; i<tailleAngle; i++)
        {
            somme += tableauAngle[i];
            tableauAngle[i] = 0;
        }

        angleLive = somme/tailleAngle;
        tailleAngle = 0;

        if (j >= 1)
        {
            omega = (angleLive-anglePrec)*1000/dtMs_;
        }
        j++;
        anglePrec = angleLive;
    }
}

void oscillation::setPosSapin(double posSapin)
{
    maxPos = posSapin-0.1163;
}




void oscillation::setShits(float _accel, float _pointeActivite, int _angleMin, float _noSlipCommand)
{
    Accel = _accel;
    pointeActivite = _pointeActivite;
    angleMin = _angleMin;
    noSlipCommand = _noSlipCommand;
}