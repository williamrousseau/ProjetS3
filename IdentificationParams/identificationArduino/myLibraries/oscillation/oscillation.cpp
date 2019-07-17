/*
Projet S3 GRO
Class to control a PID
@author Emiles
@version 1.0 23/04/2019
*/

#include <Arduino.h>
#include "sequencement.h"

oscillation::oscillation()
{
    tailleAngle = 0;
    capaciteAngle = 10;
    tailleMoyenne = 0;
    capaciteMoyenne = 500;
    omega = 0;
    dtMs = 1; //période de 1 ms
    tableauMoyenne = new float [capaciteMoyenne];
}

void oscillation::enable()
{
    measureTime = millis() + dtMs;
}

float oscillation::commandeOscillation(int angle, float commandePrec)
{
    double commande = commandePrec;
    if (vitesseAngulaire(angle) < epsilon && vitesseAngulaire(angle) > -1*epsilon)
    {
        if (angle > 0)
        {
            commande = -0.99;
        }
        else
        {
            commande = 0.99;
        }
    }
    return commande;
}

float oscillation::vitesseAngulaire(int angle)
{
    if (millis() >= measureTime)
    {
        tableauAngle[tailleAngle] = angle;
        tailleAngle++;
        float somme = 0;

        if (tailleAngle == capaciteAngle)
        {
            for (int i=0; i<tailleAngle; i++)
            {
                somme = tableauAngle[i];
                tableauAngle[i] = 0;
            }

            tableauMoyenne[tailleMoyenne] = somme/tailleAngle;
            tailleMoyenne++;
            tailleAngle = 0;

            if (tailleMoyenne >= capaciteMoyenne)
            {
                float tempo[capaciteMoyenne*2];
                for(int i=0; i<tailleMoyenne; i++)
                {
                    tempo[i] = tableauMoyenne[i];
                    tableauMoyenne[i]=0;
                }
                capaciteMoyenne *= 2;
                delete tableauMoyenne;
                tableauMoyenne = tempo;
            }
            if (tailleMoyenne >= 2)
            {
                omega = (tableauMoyenne[tailleMoyenne-1]-tableauMoyenne[tailleMoyenne-2])*1000/dtMs;
            }
        }
    }
    return omega;
}