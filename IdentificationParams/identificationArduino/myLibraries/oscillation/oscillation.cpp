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
    dtMs_ = 1; //période de 1 ms
    epsilon = 1;
    enabled = 0;
    lastCommand = 0;
    j = 0;
    anglePrec = 0;
    angleZero = 0;
    currentDifference = 0;
    sens = 0;
    angleMax = 0;
    startUp = 1;
}

void oscillation::enable()
{
    measureTime_[0] = millis() + dtMs_;
    enabled = 1;
    angleZero = measurementFunc();
    Serial.println("enabled");
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
        sens = 1;
        angleMax = abs(angle - angleZero);
        startUp = 0;
        commande = 0.5;
        commandFunc(commande);

    }
    
    else if (omega < 0)
    {
        Serial.println("--------------");
        if(sens == -1)
        {   
            angleMax = abs(angle - angleZero);
        }
        sens = 1;
        currentDifference = abs(angle - angleZero);
        if (currentDifference )
        commande = (angleMax - currentDifference)/angleMax;
        commandFunc(commande); 
    }
    else 
    {
        Serial.println("OOOOOOOOOOOO");
        if(sens == 1)
        {
            angleMax = abs(angle - angleZero);
        }
        sens = -1;
        currentDifference = abs(angle - angleZero);
        commande = -(angleMax - currentDifference)/angleMax;
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
