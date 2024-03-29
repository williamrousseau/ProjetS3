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

#define MAGPIN          32          // Port numerique pour electroaimant
#define POTPIN          A5          // Port analogique pour le potentiometre
#define BUMPERAVANT     38          // if(digitalRead(BUMPERAVANT))
#define BUMPERARRIERE   9           // if(digitalRead(BUMPERARRIERE))

#define PASPARTOUR      64          // Nombre de pas par tour du moteur
#define RAPPORTVITESSE  50          // Rapport de vitesse du moteur

#define INITTOE               1     // Variable d'etat pour le cas d'initialisation du sequencement
#define OSCILLATION           2     // Variable d'etat pour le cas d'oscillement du sequencement
#define PREPARE1TOURBILLON    3
#define PREPARE2TOURBILLON    4
#define STUFAITPASDTOURBILLON 5     // Variable d'etat pour le cas d'acceleration du sequencement
#define CALMETOE              6     // Variable d'etat pour le cas de ralentissement du sequencement
#define GORESET               7 
#define GOGETMOREBREAD        9     // Variable d'etat pour le cas de retour du sequencement

#define INITTOE2              101
#define COMP                  102
#define CALMETOE2             103
#define RETOUR                104
#define EMILEAPASDEMOTRICITE  105
#define RETOURCALME           106

/*---------------------------- variables globales ---------------------------*/

ArduinoX AX_;                       // objet arduinoX
MegaServo servo_;                   // objet servomoteur
VexQuadEncoder vexEncoder_;         // objet encodeur vex
IMU9DOF imu_;                       // objet imu
doublePID pid_double;                     // objet PID
doublePID pid_double2;
doublePID pid_double3;
doublePID pid_pendule;                     // objet PID
doublePID pid_position;                     // objet PID
doublePID pid_retour;
doublePID pid_reset;
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
double distance_obstacle_;
double energy_;
bool magnet_on_;
bool test_mode_, comp_mode_;
bool readyTOchange_;
int etat_, calmeBRO_;
bool CALME;
bool lecriss_ = false;
unsigned long nextTime;

/*------------------------- Prototypes de fonctions -------------------------*/

void timerCallback();
void sendMsg(); 
void readMsg();
void serialEvent();

// Fonctions pour le PID
double PIDmeasurementPos();
double PIDmeasurementAngle();
double PIDmeasurementAngleNoflip();
void PIDcommand(double cmd);
void PIDgoalReached1();
void PIDgoalReached2();

// Nos fonctions ajoutees
double get_energy();
void manage_state(bool run_);

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

  if(digitalRead(BUMPERARRIERE))
{
  AX_.resetEncoder(0);
  AX_.resetEncoder(1);
}

  //Electroaimant
  pinMode(MAGPIN,OUTPUT);
  //digitalWrite(MAGPIN,HIGH);

  // Initialisation du PID double
  pid_double2.setGains(5, 0.001 ,0.0001, -0.5, -0, -0.1/*5, 0 ,0.0001 , 10, 0, 1*/);
  pid_double2.setWeight(0.7, 0.3);
    // Attache des fonctions de retour
    pid_double2.setMeasurementFunc(PIDmeasurementPos, PIDmeasurementAngleNoflip/*Noflip*/);
    pid_double2.setCommandFunc(PIDcommand);
    pid_double2.setAtGoalFunc(PIDgoalReached1, PIDgoalReached2);
  pid_double2.setEpsilon(0.005, 13);
  pid_double2.setPeriod(10);
  pid_double2.setGoal(0.5, 0);
  pid_double2.enable();

  pid_double.setGains(5, 0.001 ,0.0001, -0.3, -0, -0.1/*5, 0 ,0.0001 , 10, 0, 1*/);
  pid_double.setWeight(0.98, 0.02);
    // Attache des fonctions de retour
    pid_double.setMeasurementFunc(PIDmeasurementPos, PIDmeasurementAngleNoflip/*Noflip*/);
    pid_double.setCommandFunc(PIDcommand);
    pid_double.setAtGoalFunc(PIDgoalReached1, PIDgoalReached2);
  pid_double.setEpsilon(0.005, 13);
  pid_double.setPeriod(10);
  pid_double.setGoal(1.375, 0);
  pid_double.enable();

  pid_double3.setGains(5, 0.001 ,0.0001, -0.2, -0, -0.1/*5, 0 ,0.0001 , 10, 0, 1*/);
  pid_double3.setWeight(0.95, 0.05);
    // Attache des fonctions de retour
    pid_double3.setMeasurementFunc(PIDmeasurementPos, PIDmeasurementAngleNoflip/*Noflip*/);
    pid_double3.setCommandFunc(PIDcommand);
    pid_double3.setAtGoalFunc(PIDgoalReached1, PIDgoalReached2);
  pid_double3.setEpsilon(0.005, 13);
  pid_double3.setPeriod(10);
  pid_double3.setGoal(0.20, 0);
  pid_double3.enable();

  // Initialisation du PID double
  pid_pendule.setGains(5, 0.001 ,0.0001,-0.5, -0, -0.1);
  pid_pendule.setWeight(0,0.025);
    // Attache des fonctions de retour
    pid_pendule.setMeasurementFunc(PIDmeasurementPos, PIDmeasurementAngleNoflip/*Noflip*/);
    pid_pendule.setCommandFunc(PIDcommand);
    pid_pendule.setAtGoalFunc(PIDgoalReached1, PIDgoalReached2);
  pid_pendule.setEpsilon(0.005, 9);
  pid_pendule.setPeriod(10);
  pid_pendule.setGoal(0, 0);
  pid_pendule.enable();
  
  // Initialisation du PID position
  pid_position.setGains(5, 0.001 ,0.0001,-0.5, -0, -0.1);
  pid_position.setWeight(1,0);
    // Attache des fonctions de retour
    pid_position.setMeasurementFunc(PIDmeasurementPos, PIDmeasurementAngleNoflip/*Noflip*/);
    pid_position.setCommandFunc(PIDcommand);
    pid_position.setAtGoalFunc(PIDgoalReached1, PIDgoalReached2);
  pid_position.setEpsilon(0.005, 9);
  pid_position.setPeriod(10);
  pid_position.setGoal(1.375, 0);
  pid_position.enable();

  // Initialisation du PID retour
  pid_retour.setGains(5, 0.001 ,0.0001,-0.5, -0, -0.1);
  pid_retour.setWeight(1,0);
    // Attache des fonctions de retour
    pid_retour.setMeasurementFunc(PIDmeasurementPos, PIDmeasurementAngleNoflip/*Noflip*/);
    pid_retour.setCommandFunc(PIDcommand);
    pid_retour.setAtGoalFunc(PIDgoalReached1, PIDgoalReached2);
  pid_retour.setEpsilon(0.005, 9);
  pid_retour.setPeriod(10);
  pid_retour.setGoal(0.15, 0);
  pid_retour.enable();

  // Initialisation du PID reset position
  pid_reset.setGains(5, 0.001 ,0.0001,-0.5, -0, -0.1);
  pid_reset.setWeight(1,0);
    // Attache des fonctions de retour
    pid_reset.setMeasurementFunc(PIDmeasurementPos, PIDmeasurementAngleNoflip/*Noflip*/);
    pid_reset.setCommandFunc(PIDcommand);
    pid_reset.setAtGoalFunc(PIDgoalReached1, PIDgoalReached2);
  pid_reset.setEpsilon(0.005, 9);
  pid_reset.setPeriod(10);
  pid_reset.setGoal(0, 0);
  pid_reset.enable();


  oscille.setCommandFunc(PIDcommand);
  oscille.setMeasurementFunc1(PIDmeasurementAngleNoflip);
  oscille.setMeasurementFunc2(PIDmeasurementPos);
  oscille.enable();

  oscille.setCommandFunc(PIDcommand);
  oscille.setMeasurementFunc1(PIDmeasurementAngleNoflip);
  oscille.setMeasurementFunc2(PIDmeasurementPos);
  oscille.enable();

  // Initialisation des variables
  readyTOchange_ = false;
  etat_ = 0;
  run_ = false;
  CALME = false;
  calmeBRO_ = 0;
}


/* Boucle principale (infinie)*/
void loop() {
  

  if(shouldRead_){
    readMsg();
  }
  if(shouldSend_){
    sendMsg();
  }

  
  //pid_double.run();                                                                   //OVERWRITE POUR TESTS
if (test_mode_)
{
  if (etat_ == INITTOE && readyTOchange_)
  {
    etat_ = OSCILLATION;
    readyTOchange_ = false;
    CALME = false;
    //oscille.setShits(0.48, 0.8, 35, 0.5);
  }
  

  if(etat_ == OSCILLATION && readyTOchange_){
    etat_ = PREPARE1TOURBILLON;
    readyTOchange_ = false;
    // Reste des initialisation pour le prochain état
  }

  if(etat_ == PREPARE1TOURBILLON && readyTOchange_){
    etat_ = PREPARE2TOURBILLON;
    readyTOchange_ = false;
    // Reste des initialisation pour le prochain état
  }

  if(etat_ == PREPARE2TOURBILLON && readyTOchange_){
    etat_ = STUFAITPASDTOURBILLON;
    readyTOchange_ = false;
    // Reste des initialisation pour le prochain état
  }

  if(etat_ == STUFAITPASDTOURBILLON && readyTOchange_){
    etat_ = GORESET;
    readyTOchange_ = false;
    // Reste des initialisation pour le prochain état
  }

  if(etat_ == GORESET && readyTOchange_){
    etat_ = CALMETOE;
    nextTime = millis() + 7500;
    readyTOchange_ = false;
    CALME = false;
    // Reste des initialisation pour le prochain état
  }

  if(etat_ == CALMETOE && readyTOchange_){
    etat_ = GOGETMOREBREAD;
    readyTOchange_ = false;
    // Reste des initialisation pour le prochain état
  }

  if(etat_ == GOGETMOREBREAD && readyTOchange_){
    etat_ = INITTOE;
    readyTOchange_ = false;
    digitalWrite(MAGPIN, LOW);
    // Reste des initialisation pour le prochain état
  }

  if (run_)
  {

    /*unsigned long measureTime_ = 0;
    bool timerSet_ = false;*/
    switch (etat_)
    {
      case INITTOE:
      AX_.setMotorPWM(0, -0.2);
      AX_.setMotorPWM(1, -0.2);
      if(digitalRead(BUMPERARRIERE)){          
          AX_.setMotorPWM(0, 0);
          AX_.setMotorPWM(1, 0);
          AX_.resetEncoder(0);
          AX_.resetEncoder(1);          
          digitalWrite(MAGPIN,HIGH);
          pid_reset.run();
          pid_reset.disable1();
          readyTOchange_ = true;
        }  
      break;

      case OSCILLATION:   
        oscille.run();        
        if (PIDmeasurementAngleNoflip() > 147){
          readyTOchange_ = true;
        }  
       break;

       case PREPARE1TOURBILLON:
       oscille.run();
       if(PIDmeasurementAngleNoflip() < -90) {
        readyTOchange_ = true;  }
        break;

       case PREPARE2TOURBILLON:
       oscille.run();
       if(PIDmeasurementAngleNoflip() > 40)
          readyTOchange_ = true;   
       break;
    
      case STUFAITPASDTOURBILLON:
        pid_position.run();
        oscille.disable();        
        if(pid_position.isAtGoal1()) {     
          readyTOchange_ = true;
          pid_position.disable1();  
          }                              
        break;     

      case GORESET:
              AX_.setMotorPWM(0, 0.1);
              AX_.setMotorPWM(1, 0.1);
                if(digitalRead(BUMPERAVANT)){          
                  AX_.setMotorPWM(0, 0);
                  AX_.setMotorPWM(1, 0);            
                  readyTOchange_ = true;           
              }
            break;

      case CALMETOE:
        //pid_double.enable();
        pid_double.run();        
        if(millis()>nextTime){
          digitalWrite(MAGPIN, LOW);
          readyTOchange_ = true;
          pid_double.disableAll();
          pid_position.enable();
        }
        break;

      

      case GOGETMOREBREAD:
        digitalWrite(MAGPIN, LOW);
        readyTOchange_ = true;
        oscille.enable();
        pid_double.enable(); 
        //magnet_on_ = false;
        /*pid_retour.run();
        if(pid_retour.isAtGoal1() + 0.9){           
           digitalWrite(MAGPIN, HIGH);
        }   
        if(pid_retour.isAtGoal1()){           
                     
        } */         
        break;
    }
  }
}
if (comp_mode_)
{
  if (etat_ == INITTOE2 && readyTOchange_)
  {
    etat_ = COMP;
    readyTOchange_ = false;
    CALME = false;
    //oscille.setShits(01.5, 1, 35, 0.9);
    //oscille.setPosSapin(0.6);
  }
  

  if(etat_ == COMP && readyTOchange_){
    etat_ = CALMETOE2;
    nextTime = millis() + 300;
    readyTOchange_ = false;
    // Reste des initialisation pour le prochain état
  }

  if(etat_ == CALMETOE2 && readyTOchange_){
    etat_ = RETOUR;
    digitalWrite(MAGPIN, LOW);
    readyTOchange_ = false;
    // Reste des initialisation pour le prochain état
  }
  if(etat_ == RETOUR && readyTOchange_){
    etat_ = EMILEAPASDEMOTRICITE;
    nextTime = millis() + 1500;
    readyTOchange_ = false;
    // Reste des initialisation pour le prochain état
  }

  if(etat_ == EMILEAPASDEMOTRICITE && readyTOchange_){
    etat_ = RETOURCALME;
    readyTOchange_ = false;
    // Reste des initialisation pour le prochain état
  }

  if(etat_ == RETOURCALME && readyTOchange_){
    etat_ = INITTOE2;
    nextTime = millis() + 1000;
    readyTOchange_ = false;
    // Reste des initialisation pour le prochain état
  }


  if (run_)
  {

    /*unsigned long measureTime_ = 0;
    bool timerSet_ = false;*/
    switch (etat_)
    {
      case INITTOE2:
      AX_.setMotorPWM(0, -0.2);
      AX_.setMotorPWM(1, -0.2);
      digitalWrite(MAGPIN,HIGH);
      if(digitalRead(BUMPERARRIERE)){          
          AX_.setMotorPWM(0, 0);
          AX_.setMotorPWM(1, 0);
          AX_.resetEncoder(0);
          AX_.resetEncoder(1); 
          if (millis() > nextTime)
          {
            readyTOchange_ = true;
          }
          }
      
          
      break;

      case COMP:       
        PIDcommand(1);
        if (PIDmeasurementPos()>0.5)
        {
          PIDcommand(0);
          readyTOchange_ = true;
        }
       break;

      case CALMETOE2:
      pid_double2.run();
      if (millis() > nextTime)
      { 
        digitalWrite(MAGPIN, LOW);
        readyTOchange_ = true;
      }
        break;

       case RETOUR:
       PIDcommand(-1);
       if(PIDmeasurementPos() < 0.20)
        readyTOchange_ = true;   
       break;
    
      case EMILEAPASDEMOTRICITE:
        pid_double3.run();
       if(millis() > nextTime)
        readyTOchange_ = true;   
       break;

      case RETOURCALME:
        PIDcommand(-0.5);
        digitalWrite(MAGPIN,HIGH);
        if(digitalRead(BUMPERARRIERE))
          AX_.resetEncoder(0);
          AX_.resetEncoder(1);
          PIDcommand(0);
          readyTOchange_ = true;                       
        break;     
      
    }
  }
}
  // mise a jour des chronometres
  timerSendMsg_.update();
  timerPulse_.update();

  /*if(millis() < timeout)
  {
    oscille.run();
  }
  if(millis() > timeout)
  {
    AX_.setMotorPWM(0, 0);
    AX_.setMotorPWM(1, 0);
  }*/

  

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

void sendMsg(){
  /* Envoit du message Json sur le port seriel */
  StaticJsonDocument<500> doc;
  // Elements du message
  doc["Time"] = millis();
  doc["In_run"] = run_;
  doc["Magnet_on"] = magnet_on_;
  doc["Distance_obstacle"] = distance_obstacle_;
  doc["Voltage"] = AX_.getVoltage();
  doc["Current"] = AX_.getCurrent(); 
  doc["Power"] = AX_.getVoltage() * AX_.getCurrent();
  doc["Energy (Ws)"] = get_energy(); 
  doc["Test_mode"] = test_mode_;
  doc["Competition_mode"] = comp_mode_;
  doc["Position"] = PIDmeasurementPos() * 100;
  doc["Etat"] = etat_;
  doc["Angle"] = PIDmeasurementAngleNoflip();
  doc["Nb_fois_0"] = calmeBRO_;
  doc["Le_criss"] = digitalRead(BUMPERAVANT);


  // Serialisation
  serializeJson(doc, Serial);
  // Envoit
  Serial.println();
  shouldSend_ = false;
}

double get_energy(){
  energy_ = energy_ + (AX_.getVoltage() * (AX_.getCurrent()) * UPDATE_PERIODE*1000);
  return energy_;
}

void manage_state(bool run_){
  if (run_ && test_mode_)
  {
    etat_ = INITTOE;
  }
  if (run_ && comp_mode_)
  {
    etat_ = INITTOE2;
  }
  if (!run_)
  {
    etat_ = 0;
    AX_.setMotorPWM(0, 0);
    AX_.setMotorPWM(1, 0);
  }
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

  parse_msg = doc["Distance_obstacle"];
  if(!parse_msg.isNull()){
     distance_obstacle_ = doc["Distance_obstacle"];
     oscille.setPosSapin(distance_obstacle_);
  }
  parse_msg = doc["In_run"];
  if(!parse_msg.isNull()){
    run_ = doc["In_run"];
    manage_state(run_);
  }
  /* parse_msg = doc["Magnet_on"];
  if(!parse_msg.isNull()){
    magnet_on_ = doc["Magnet_on"];
    if (magnet_on_)
  {
    digitalWrite(MAGPIN, HIGH);
  }
  if(!magnet_on_)
  {
    digitalWrite(MAGPIN, LOW);
  }
  }*/
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
double PIDmeasurementPos(){ //Position du chariot
  double position1 = AX_.readEncoder(0)*kgear*WheelR*PI*2/3200;
  double position2 = AX_.readEncoder(1)*kgear*WheelR*PI*2/3200;
  double position = (position1 + position2)/2;
  return position;
}
double PIDmeasurementAngle(){ //Position du pendule
  return vexEncoder_.getCount()*2;
}

double PIDmeasurementAngleNoflip(){ //Position du pendule
  int32_t angle_deg = PIDmeasurementAngle();

  //Retourne des angles entre -360 et 360 grâce à :
  while(angle_deg < -184){
    angle_deg = angle_deg + 360;
    //int tours ++ 
  }
  while(angle_deg > 184){
    angle_deg = angle_deg - 360;
  }
  return angle_deg;
}



/* Dépend des enables de PID
Sortie dépendante des deux PIDS */
void PIDcommand(double cmd){
  AX_.setMotorPWM(0, cmd);
  AX_.setMotorPWM(1, cmd);
}
void PIDgoalReached1(){
  //pid_double.disable1();
}
void PIDgoalReachedDouble(){
    
}
void PIDgoalReached2(){
  calmeBRO_ = calmeBRO_ + 1;
  if(calmeBRO_ > 300){
     CALME = true;
     //pid_double.disableAll();
     calmeBRO_ = 0;
  }
}