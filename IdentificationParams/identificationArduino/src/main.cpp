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
doublePID pid_;                          // objet PID

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
const double pi = 3.14159265359;
const double WheelR = 0.025;

/*------------------------- Prototypes de fonctions -------------------------*/

void timerCallback();
void startPulse();
void endPulse();
void sendMsg(); 
void readMsg();
void serialEvent();

// Fonctions pour le PID
double PIDmeasurement1();
double PIDmeasurement2();
void PIDcommand(double cmd);
void PIDgoalReached1();
void PIDgoalReached2();

/*---------------------------- fonctions "Main" -----------------------------*/

void setup() {
  Serial.begin(BAUD);               // initialisation de la communication serielle
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
  
  // Initialisation du PID 1
  pid_.setGains(5, 0 ,0.0001, 10, 0, 1);       //gains bidons
  pid_.setWeight(1, 0);                       //pondérations bidons
  //pid_.setWeight(1-0.025,0.025);
    // Attache des fonctions de retour
    pid_.setMeasurementFunc(PIDmeasurement1, PIDmeasurement2);
    pid_.setCommandFunc(PIDcommand);
    pid_.setAtGoalFunc(PIDgoalReached1, PIDgoalReached2);
  pid_.setEpsilon(0.001, 0.001);                 //tolerances bidons
  pid_.setPeriod(10);
  pid_.setGoal(0.010,0);
  pid_.enable();
}

/* Boucle principale (infinie)*/
void loop() {

  if(shouldRead_){
    readMsg();
  }
  if(shouldSend_){
    sendMsg();
  }
  if(shouldPulse_){
    startPulse();
  }

  // mise a jour des chronometres
  timerSendMsg_.update();
  timerPulse_.update();
  // mise à jour du PID
  //pid_.run();


  pinMode(MAGPIN,OUTPUT);
  
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

  doc["time"] = millis();
  doc["potentiometre"] = analogRead(POTPIN);
  doc["encVex"] = vexEncoder_.getCount();
  doc["goal1"] = pid_.getGoal1();
  doc["goal2"] = pid_.getGoal2();
  doc["motorPos"] = PIDmeasurement1();
  doc["pendulumPos"] = PIDmeasurement2();
  doc["voltage"] = AX_.getVoltage();
  doc["current"] = AX_.getCurrent(); 
  doc["pulsePWM"] = pulsePWM_;
  doc["pulseTime"] = pulseTime_;
  doc["inPulse"] = isInPulse_;
//  doc["accelX"] = imu_.getAccelX();
//  doc["accelY"] = imu_.getAccelY();
//  doc["accelZ"] = imu_.getAccelZ();
//  doc["gyroX"] = imu_.getGyroX();
//  doc["gyroY"] = imu_.getGyroY();
//  doc["gyroZ"] = imu_.getGyroZ();
  doc["isGoal1"] = pid_.isAtGoal1();
  doc["isGoal2"] = pid_.isAtGoal2();

  // Serialisation
  serializeJson(doc, Serial);
  // Envoit
  Serial.println();
  shouldSend_ = false;
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
  parse_msg = doc["pulsePWM"];
  if(!parse_msg.isNull()){
     pulsePWM_ = doc["pulsePWM"].as<float>();
  }

  parse_msg = doc["pulseTime"];
  if(!parse_msg.isNull()){
     pulseTime_ = doc["pulseTime"].as<float>();
  }

  parse_msg = doc["pulse"];
  if(!parse_msg.isNull()){
     shouldPulse_ = doc["pulse"];
  }
}


// Fonctions pour le PID
double PIDmeasurement1(){ //Position du chariot
  double position1 = AX_.readEncoder(1)*kgear*WheelR*pi*2/3200;
  double position2 = AX_.readEncoder(2)*kgear*WheelR*pi*2/3200;
  double position = (position1 + position2)/2;
  return position;
}
double PIDmeasurement2(){ //Position du pendule
  return analogRead(POTPIN);
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
  // To do
}