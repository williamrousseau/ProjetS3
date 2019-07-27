/* 
 * GRO 302 - Conception d'un robot mobile
 * Code de démarrage
 * Auteurs: Jean-Samuel Lauzon     
 * date: 1 mai 2019
*/

/*------------------------------ Librairies ---------------------------------*/
#include <LibS3GRO.h>
#include <ArduinoJson.h>
#include "doublePID.h" //Librairie gérant 2 PIDs
#include "oscillation.h" 
#include "abc.h"
#include "sequencement.h"
#include "obstacle.h"
/*------------------------------ Constantes ---------------------------------*/

#define BAUD            115200      // Frequence de transmission serielle
#define UPDATE_PERIODE  200         // Periode (ms) d'envoie d'etat general

#define MAGPIN           8          // Port numerique pour electroaimant
#define POTPIN          A5          // Port analogique pour le potentiometre

#define PASPARTOUR      64          // Nombre de pas par tour du moteur
#define RAPPORTVITESSE  50          // Rapport de vitesse du moteur

/*---------------------------- variables globales ---------------------------*/

ArduinoX AX_;                       // objet arduinoX
MegaServo servo_;                   // objet servomoteur
VexQuadEncoder vexEncoder_;         // objet encodeur vex
IMU9DOF imu_;                       // objet imu
doublePID pid_;                     // objet PID
oscillation oscille;

volatile bool shouldSend_ = false;  // drapeau prêt à envoyer un message
volatile bool shouldRead_ = false;  // drapeau prêt à lire un message
volatile bool shouldPulse_ = false; // drapeau pour effectuer un pulse
volatile bool isInPulse_ = false;   // drapeau pour effectuer un pulse

SoftTimer timerSendMsg_;            // chronometre d'envoie de messages
SoftTimer timerPulse_;              // chronometre pour la duree d'un pulse

uint16_t pulseTime_ = 0;            // temps dun pulse en ms
float pulsePWM_ = 0;                // Amplitude de la tension au moteur [-1,1]


float Axyz[3];                      // tableau pour accelerometre
float Gxyz[3];                      // tableau pour giroscope
float Mxyz[3];                      // tableau pour magnetometre

const double kgear = 2;
const double WheelR = 0.025;
unsigned long timeout;
Obstacle Sapin1;
bool run_;
int nb_obstacle_;
double energy_;
bool magnet_on_;
bool test_mode_, comp_mode_;
/*------------------------- Prototypes de fonctions -------------------------*/

void timerCallback();
void startPulse();
void endPulse();
void sendMsg(); 
void readMsg();
void serialEvent();
double get_energy();

// Fonctions pour le PID
double PIDmeasurement1();
double PIDmeasurement2();
void PIDcommand(double cmd);
void PIDgoalReached1();
void PIDgoalReached2();

/*---------------------------- fonctions "Main" -----------------------------*/

void setup() {
  Serial.begin(BAUD);               // initialisation de la communication serielle
  Serial.println("ALLO");
    AX_.init();                       // initialisation de la carte ArduinoX 
  imu_.init();                      // initialisation de la centrale inertielle
  vexEncoder_.init(2,3);            // initialisation de l'encodeur VEX
  // attache de l'interruption pour encodeur vex
  attachInterrupt(vexEncoder_.getPinInt(), []{vexEncoder_.isr();}, FALLING);
  
  // Chronometre envoie message
  timerSendMsg_.setDelay(UPDATE_PERIODE);
  timerSendMsg_.setCallback(timerCallback);
  timerSendMsg_.enable();

  // Chronometre duration pulse
  timerPulse_.setCallback(endPulse);

  //electroaimant
  pinMode(MAGPIN,OUTPUT);

  // Initialisation du PID 1
  pid_.setGains(5, 0 ,0.0001, 10, 0, 1);       //gains bidons
  pid_.setWeight(1, 0);                       //pondérations bidons
  //pid_.setWeight(1-0.025,0.025);
    // Attache des fonctions de retour
    pid_.setMeasurementFunc(PIDmeasurement1, PIDmeasurement2);
    pid_.setCommandFunc(PIDcommand);
    pid_.setAtGoalFunc(PIDgoalReached1, PIDgoalReached2);
  pid_.setEpsilon(0.001, 0.001);                 //tolerances bidons
  pid_.setPeriod(200);
  pid_.setGoal(2,0);
  pid_.enable();
  Serial.println("avant");
  oscille.setCommandFunc(PIDcommand);
  Serial.println("apres");
  oscille.setMeasurementFunc(PIDmeasurement2);
  oscille.enable();
  timeout = millis() + 100000;
}


/* Boucle principale (infinie)*/
void loop() {
  
   if(shouldRead_){
    readMsg();
  }
  if(shouldSend_){
    sendMsg();
  }
  /* if(shouldPulse_){
    startPulse();
  }*/

  // mise a jour des chronometres
  timerSendMsg_.update();
  timerPulse_.update();
  // mise à jour du PID
  //pid_.run();
  
  if(millis() < timeout)
  {
    oscille.run();
  }
  if(millis() > timeout)
  {
    AX_.setMotorPWM(0, 0);
    AX_.setMotorPWM(1, 0);
  }

  //digitalWrite(MAGPIN,HIGH);
  

  //********************************TESTS**************************************
   /*   
    switch (1)
  {
  case 1:         //T*************TESTS Obstacle************************

  Sapin1.setdistance(32);
  Sapin1.setdistance_init(10);
  Sapin1.sethauteur(17);
  Sapin1.sethauteur_init(28);
  Serial.println(Sapin1.getdistance());
  Serial.println(Sapin1.gethauteur());
    break;
  
  default:
            Serial.println("Erreur");  
    break; 
    
  }*/
 
}

/*---------------------------Definition de fonctions ------------------------*/

void serialEvent(){shouldRead_ = true;}

void timerCallback(){shouldSend_ = true;}

void startPulse(){
  /* Demarrage d'un pulse */
  timerPulse_.setDelay(pulseTime_);
  timerPulse_.enable();
  timerPulse_.setRepetition(1);
  AX_.setMotorPWM(0, pulsePWM_);
  AX_.setMotorPWM(1, pulsePWM_);
  shouldPulse_ = false;
  isInPulse_ = true;
}

void endPulse(){
  /* Rappel du chronometre */
  AX_.setMotorPWM(0,0);
  AX_.setMotorPWM(1,0);
  timerPulse_.disable();
  isInPulse_ = false;
}


void sendMsg(){
  /* Envoit du message Json sur le port seriel */
  StaticJsonDocument<500> doc;
  // Elements du message

  doc["Time"] = millis();
  doc["Potentiometre"] = analogRead(POTPIN);
  doc["In_run"] = run_;
  doc["Magnet_on"] = magnet_on_;
  doc["Nb_obstacle"] = nb_obstacle_;
  doc["Voltage"] = AX_.getVoltage();
  doc["Current"] = AX_.getCurrent(); 
  doc["Power"] = AX_.getVoltage() * AX_.getCurrent();
  doc["Energy (Ws)"] = get_energy(); 
  doc["Test_mode"] = test_mode_;
  doc["Competition_mode"] = comp_mode_;


  // Serialisation
  serializeJson(doc, Serial);
  // Envoit
  Serial.println();
  shouldSend_ = false;
}

double get_energy(){
  energy_ = energy_ + (AX_.getVoltage() * AX_.getCurrent() * UPDATE_PERIODE/1000);
  return energy_;
}

void readMsg(){
  // Lecture du message Json
  StaticJsonDocument<500> doc;
  JsonVariant parse_msg;

  // Lecture sur le port Seriel
  DeserializationError error = deserializeJson(doc, Serial);
  shouldRead_ = false;

  // Si erreur dans le message
  if (error) {
    Serial.print("deserialize() failed: ");
    Serial.println(error.c_str());
    return;
  }
  
  // Analyse des éléments du message message
  /*parse_msg = doc["pulsePWM"];
  if(!parse_msg.isNull()){
     pulsePWM_ = doc["pulsePWM"].as<float>();
  }

  parse_msg = doc["pulseTime"];
  if(!parse_msg.isNull()){
     pulseTime_ = doc["pulseTime"].as<float>();
  }*/

  parse_msg = doc["Nb_obstacle"];
  if(!parse_msg.isNull()){
     nb_obstacle_ = doc["Nb_obstacle"];
  }
  parse_msg = doc["In_run"];
  if(!parse_msg.isNull()){
    run_ = doc["In_run"];
  }
  parse_msg = doc["Magnet_on"];
  if(!parse_msg.isNull()){
    magnet_on_ = doc["Magnet_on"];
  }
  parse_msg = doc["Competition_mode"];
  if(!parse_msg.isNull()){
     comp_mode_= doc["Competition_mode"];
  }
  parse_msg = doc["Test_mode"];
  if(!parse_msg.isNull()){
     test_mode_= doc["Test_mode"];
  }
}


// Fonctions pour le PID
double PIDmeasurement1(){ //Position du chariot
  double position1 = AX_.readEncoder(1)*kgear*WheelR*PI*2/3200;
  double position2 = AX_.readEncoder(2)*kgear*WheelR*PI*2/3200;
  double position = (position1 + position2)/2;
  return position;
}
double PIDmeasurement2(){ //Position du pendule
  return vexEncoder_.getCount()*2;
}

/* Dépend des enables de PID
Sortie dépendante des deux PIDS */
void PIDcommand(double cmd){
  AX_.setMotorPWM(0, cmd);
  AX_.setMotorPWM(1, cmd);
}
void PIDgoalReached1(){
  pid_.disable();
}
void PIDgoalReached2(){
  pid_.disable();
}