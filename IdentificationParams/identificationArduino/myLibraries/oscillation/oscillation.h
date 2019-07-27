/*
Projet S3 GRO
Class to control a PID
@author Emiles
@version 1.0 23/04/2019
*/

#ifndef oscillation_H_
#define oscillation_H_

#include <Arduino.h>
#include <LibS3GRO.h>

class oscillation
{
  public:
    oscillation();
    void commandeOscillation(double angle);

    void setMeasurementFunc(double (*f)()){measurementFunc = f;};
    void setCommandFunc(void (*f)(double)){commandFunc = f;};

    void enable();
    void vitesseAngulaire(double angle);
    void run();


  private:
    int j;
    float anglePrec;
    float tableauAngle[10];
    int tailleAngle;
    int capaciteAngle;
    float omega;
    float epsilon;
    unsigned long measureTime_[2]; // Mesure de time pour cette classe
    unsigned long dtMs_; // Periode entre les commandes
    bool enabled;
    float lastCommand;
    double (*measurementFunc)() = nullptr; // Measurement function
    void (*commandFunc)(double) = nullptr; // Command function
    int tailleTemps;
    int angleZero;
    int angleMax;
    int currentDifference;
    int sens;
    bool startUp;
};
#endif