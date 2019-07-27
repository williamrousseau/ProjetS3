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
    pointeActivite = 0.75;        //PARAMS
    Accel = 1.3;                 //À
    angleMin = 50;               //CHANGER
    tailleAngle = 0;
    capaciteAngle = 10;
    omega = 0;
    dtMs_ = 10; //période de 1 ms
    enabled = 0;
    lastCommand = 0;
    j = 0;
    anglePrec = 0;
    sens = 0;
    startUp = 1;
    topAngle = 0;
    belowAngle = 0;
    maxPos = 0;
    
}

void oscillation::enable()
{
    measureTime_[0] = millis() + dtMs_;
    enabled = 1;
}

void oscillation::run()
{   
    if (millis() >= measureTime_[0])
    {
        measureTime_[0] = millis() + dtMs_;
        if (enabled)
        {
            commandeOscillation(measurementFunc1());
        }
    }
}

void oscillation::commandeOscillation(double angle)
{
    float commande = 0;
    vitesseAngulaire(angle);
    if (startUp == 1)
    {
        startUp = 0;
        commande = 0.5;
        commandFunc(commande);
    }
    
    else if (omega < 0)
    {
        if(sens == -1)
        {   
            topAngle = abs(angle)*pointeActivite;
            belowAngle = -abs(angle)*pointeActivite;
            if (topAngle < angleMin) topAngle = angleMin;
            if (belowAngle > -angleMin) belowAngle = -angleMin;
        }
        sens = 1;
        if(angle < topAngle && angle > belowAngle)
        {   
            if (measurementFunc2() > maxPos)
            {
                commande = 0;
            }
            else if (angle >= 0)
            {
                commande = Accel*(topAngle-angle)/topAngle*1.5;
            }
            else if (angle < 0)
            {
                commande = Accel*(belowAngle-angle)/belowAngle;
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
    else if (omega > 0)
    {
        if(sens == 1)
        {   
            topAngle = angle*pointeActivite;
            belowAngle = -angle*pointeActivite;
            if (topAngle > -angleMin) topAngle = -angleMin;
            if (belowAngle < angleMin) belowAngle = angleMin;
        }
        sens = -1;
        if(angle > topAngle && angle < belowAngle)
        {
            if (angle <= 0)
            {
                commande = -Accel*(topAngle-angle)/topAngle*1.5;
            }
            else if (angle > 0)
            {
                commande = -Accel*(belowAngle-angle)/belowAngle;
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
    else
    {
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

void oscillation::setMaxPos(double posSapin)
{
    maxPos = posSapin-0.1;
}