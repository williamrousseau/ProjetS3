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
    tailleAngle = 0;
    capaciteAngle = 10;
    omega = 0;
    dtMs_ = 10; //période de 1 ms
    enabled = 0;
    lastCommand = 0;
    j = 0;
    anglePrec = 0;
    sens = 0;
    angleMax = 0;
    startUp = 1;
    topAngle = 0;
    belowAngle = 0;
}

void oscillation::enable()
{
    measureTime_[0] = millis() + dtMs_;
    enabled = 1;
}

void oscillation::run()
{
    //Serial.println("millis:");
    //Serial.println(millis());
    //Serial.println("measureTime_");
    //Serial.println(measureTime_[0]);
   
    if (millis() >= measureTime_[0])
    {
        //Serial.println("millis >= measureTime_");
        measureTime_[0] = millis() + dtMs_;
        if (enabled)
        {
            commandeOscillation(measurementFunc());
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
        Serial.println(angle);
        if(sens == -1)
        {   
            angleMax = abs(angle);
            topAngle = angleMax*2/3;
            belowAngle = -angleMax/2;
            if (topAngle < 15) topAngle = 15;
            if (belowAngle > -10) belowAngle = -10;
        }
        sens = 1;
        if(angle < topAngle && angle > belowAngle)
        {   
            if (angle >= 0)
            {
                commande = 2*(topAngle-angle)/topAngle;
            }
            else if (angle < 0)
            {
                commande = 2*(belowAngle-angle)/belowAngle;
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
        Serial.println(angle);
        if(sens == 1)
        {   
            angleMax = angle;
            topAngle = angleMax/2;
            belowAngle = -angleMax*2/3;
            if (topAngle > -15) topAngle = -15;
            if (belowAngle < 10) belowAngle = 10;
        }
        sens = -1;
        if(angle > topAngle && angle < belowAngle)
        {
            if (angle <= 0)
            {
                commande = -2*(topAngle-angle)/topAngle;
            }
            else if (angle > 0)
            {
                commande = -2*(belowAngle-angle)/belowAngle;
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
    
    /*Serial.println("omega");
    Serial.println(omega);*/
}
