/*
Projet S3 GRO
Class to control the obstacles
@author William
@version 1.0 07/17/2019
*/

#include "obstacle.h"

Obstacle::obstacle()
{
    dist = 0;
    haut = 0;
    dist_init = 0;
    haut_init = 0;
}


Obstacle::­~obstacle()
{
    
}


void Obstacle::setdistance_init(float distance_init)
{
    dist_init = distance_init;
}


void Obstacle::sethauteur_init(float hauteur_init)
{
    haut_init = hauteur_init;
}


void Obstacle::setdistance(float distance)
{
    dist = distance-dist_init;
}


void Obstacle::sethauteur(float hauteur)
{
    haut = hauteur-haut_init;
}


float Obstacle:: getdistance()
{
    return dist;
}

float Obstacle::gethauteur()
{
    return haut;
}