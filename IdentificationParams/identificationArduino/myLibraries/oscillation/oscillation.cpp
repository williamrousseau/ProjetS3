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
    Serial.println("Constructeur");
    tailleAngle = 0;
    capaciteAngle = 10;
    omega = 0;
    dtMs_ = 1; //période de 1 ms
    epsilon = 1;
    tailleTemps = 0;
    enabled = 0;
    lastCommand = 0;
    j = 0;
    anglePrec = 0;
    angleZero = 0;
    currentDifference = 0;
    sens = 0;
}

void oscillation::enable()
{
    measureTime_[tailleTemps] = millis() + dtMs_;
    enabled = 1;
    angleZero = measurementFunc();
    Serial.println("enabled");
}

void oscillation::run()
{
    Serial.println("millis:");
    Serial.println(millis());
    Serial.println("measureTime_");
    Serial.println(measureTime_[tailleTemps]);
   
    if (millis() >= measureTime_[tailleTemps])
    {
        tailleTemps++;
        Serial.println("millis >= measureTime_");
        measureTime_[tailleTemps] = millis() + dtMs_;
        if (enabled)
        {
            commandeOscillation(measurementFunc());
        }
    }
}

float oscillation::commandeOscillation(double angle)
{
    float commande = 0;
    vitesseAngulaire(angle);
    Serial.println("Je suis dans comandeOscillation");
    if (omega > 0)
    {
        if(sens == 1)
        {
            difference = fabs(angle - angleZero);
        }
        sens = -1;
        currentDifference = fabs(angle - angleZero);
        commande = -(difference - currentDifference)/difference;
        commandFunc(commande); 
    }
    else
    {
        if(sens == -1)
        {
            difference = fabs(angle - angleZero);
        }
        sens = 1;
        currentDifference = fabs(angle - angleZero);
        commande = (difference - currentDifference)/difference;
        commandFunc(commande); 
    }
    return commande;
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
    
    Serial.println("omega");
    Serial.println(omega);
}
