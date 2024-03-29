/*
Projet S3 GRO
Class to control the obstacles
@author William
@version 1.0 07/17/2019
*/

#ifndef Obstacle_H_
#define Obstacle_H_

#include <Arduino.h>

class Obstacle
{
  public:
    Obstacle();
    void setdistance_init(float distance_init);
    void setdistance(float distance);
    void sethauteur_init(float hauteur_init);
    void sethauteur(float hauteur);
    float getdistance();
    float gethauteur();

  private:
   float dist_init;
   float haut_init; 
   float dist;
   float haut;
};
#endif
