/*
Projet S3 GRO
Class to control a PID
@author Emiles
@version 1.0 23/04/2019
*/

#ifndef oscillation_H_
#define oscillation_H_

#include <Arduino.h>

class oscillation
{
  public:
    sequencement();
    float commandeOscillation(int angle, float commandePrec);
    void enable();
    float vitesseAngulaire(int angle);

  private:
    float* tableauMoyenne;
    float tableauAngle[];
    int tailleAngle;
    int tailleMoyenne;
    int capaciteAngle;
    int capaciteMoyenne;
    float omega;
    float epsilon = 0.1;
    unsigned long measureTime = 0; // Mesure de temps pour cette classe
    unsigned long dtMs; // Periode entre les commandes
};
#endif