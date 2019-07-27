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

    void setMeasurementFunc1(double (*f)()){measurementFunc1 = f;};
    void setMeasurementFunc2(double (*f)()){measurementFunc2 = f;};
    void setCommandFunc(void (*f)(double)){commandFunc = f;};

    void enable();
    void vitesseAngulaire(double angle);
    void run();
    void setMaxPos(double posSapin);


  private:
    int j;
    float anglePrec;
    float tableauAngle[10];
    int tailleAngle;
    int capaciteAngle;
    float omega;
    unsigned long measureTime_[2]; // Mesure de time pour cette classe
    unsigned long dtMs_; // Periode entre les commandes
    bool enabled;
    float lastCommand;
    double (*measurementFunc1)() = nullptr; // Measurement function
    double (*measurementFunc2)() = nullptr; // Measurement function
    void (*commandFunc)(double) = nullptr; // Command function
    int angleMin;
    float pointeActivite;
    int sens;
    int topAngle;
    int belowAngle;
    double maxPos;
    bool startUp;
    float Accel;
};
#endif