/*
Projet S3 GRO
Class to control a PID
@author Jean-Samuel Lauzon
@version 1.0 23/04/2019
*/

#ifndef doublePID_H_
#define doublePID_H_

#include <Arduino.h>

class doublePID
{
  public:
    /** Default constructor
    */
    doublePID();
    
    /** Method to initialize attributes
     
    @param Kp
    Proportional constant

    @param Ki
    Integral constant

    @param Kd
    Derivative constant 
    */
    doublePID(double Kp1, double Ki1, double Kd1, double Kp2, double Ki2, double Kd2);

    /** Method to enable the object
    */
    void enable();

    /** Method to disable the object
    */
    void disable1();
    void disable2();
    void disableAll();
    
    /** Method to check if the doublePID needs to be run
    @param goal
    Desired value to converge to
    */
    void run();
    
    // Set methods
    
    /** Method to set all gains constant
    @param Kp
    Proportional constant
    @param Ki
    Integral constant
    @param Kd
    Derivative constant 
    */
    void setGains(double kp1, double ki1, double kd1, double kp2, double ki2, double kd2){
      Kp_1 = kp1; Ki_1 = ki1; Kd_1 = kd1;
      Kp_2 = kp2; Ki_2 = ki2; Kd_2 = kd2;
    };

    /** Method to set Kp
    @param Kp
    Proportional constant
    */
    void setKp(double Kp1, double Kp2){Kp_1 = Kp1; Kp_1 = Kp1;};

    /** Method to set Ki
    @param Ki
    Integral constant
    */
    void setKi(double ki1, double ki2 ){Ki_1 = ki1; Ki_2 = ki2;};

    /** Method to set Kd
    @param Kd
    Derivative constant
    */
    void setKd(double kd1, double kd2){Kd_1 = kd1; Kd_2 = kd2;};

    
    /** Method to set weightPID_1, weightPID_2
    @param wPID1, wPID2
    weight of individual PIDs
    */ 
    void setWeight(double wPID1, double wPID2);

    /** Method to set period attribute
    @param period
    Time period between iterations in ms
    */
    void setPeriod(unsigned long period){
    dtMs_ = period; 
    dt_=period/1000.0;
    };

    /** Method to set goal attribute
    @param goal
    Desired value to converge to
    */
    void setGoal(double goal1, double goal2){goal_1 = goal1;goal_2 = goal2;};


    /** Method to set measurement function
    @param f
    Parameterless function that returns a double
    (how to get the measurement)
    */
    void setMeasurementFunc(double (*f)(), double (*g)()){measurementFunc_1 = f; measurementFunc_2 = g;};

    /** Method to set measurement function
    @param f
    no retrun function with double parameter (the command)
    (what to do with the new command)
    */
    void setCommandFunc(void (*f)(double)){commandFunc_ = f;};

     /** Method to set function to run when at goal (optional)
    @param f
    parameterless, no retrun function
    (what to do when goal reached)
    */
    void setAtGoalFunc(void (*f)(),void (*g)()){atGoalFunc1 = f;atGoalFunc2 = g;};

    /** Method to set integral cumulative limit
    @param lim
    limit cumulative error
    */
    void setIntegralLim(double lim1, double lim2){eIntegralLim_1 = lim1; eIntegralLim_2 = lim2;};

    /** Method to set epsilon
    @param eps
    exit condition
    */
    void setEpsilon(double eps1,double eps2){epsilon_1 = eps1; epsilon_2 = eps2;};

    // Get methods

    /** Method to know if goal is reached
    @return true if goal is reached within epsilon
    */
    bool isAtGoal1(){return atGoal_1;}
    /** Method to know if goal is reached
    @return true if goal is reached within epsilon
    */
    bool isAtGoal2(){return atGoal_2;}

    /** Method to get the current goal registered
    @return current goal
    */
    double getGoal1(){return goal_1;}
    /** Method to get the current goal registered
    @return current goal
    */
    double getGoal2(){return goal_2;}

    /** Method to get the actual delta time
    @return actualDt_
    */
    double getActualDt(){return actualDt_;}

  private:
    /** Method to compute the command
    @param error
    error relative to the desired goal value
    */
    double computeCommand1(double error1);
    double computeCommand2(double error2);


    double Kp_1 = 0; // Proportional constant
    double Ki_1 = 0; // Integral constant
    double Kd_1 = 0; // Derivative constant

    double Kp_2 = 0; // Proportional constant
    double Ki_2 = 0; // Integral constant
    double Kd_2 = 0; // Derivative constant

    // Function pointers
    double (*measurementFunc_1)() = nullptr; // Measurement function
    double (*measurementFunc_2)() = nullptr; // Measurement function
    void (*commandFunc_)(double) = nullptr; // Command function
    void (*atGoalFunc1)() = nullptr; // Fonction called when goal is reached
    void (*atGoalFunc2)() = nullptr; // Fonction called when goal is reached

    double commandPID_1 = 0;
    double commandPID_2 = 0;
    double goal_1 = 0; // Desired state
    double goal_2 = 0; // Desired state
    bool enable_1 = false; // Enable flag
    bool enable_2 = false; // Enable flag
    bool atGoal_1 = false; // Flag to know if at goal 
    bool atGoal_2 = false; // Flag to know if at goal 
    
    double dt_; // Theoric time between 2 measurments
    unsigned long dtMs_; // Periode between commands
    unsigned long actualDt_; // Actual periode between last command
    unsigned long measureTime_ = 0; // Time for next iteration 
    unsigned long lastMeasureTime_ = 0; // Time of last iteration 



    double epsilon_1 = 5;
    double epsilon_2 = 5;
    double eIntegralLim_1 = 100;
    double eIntegralLim_2 = 100;
    double eIntegral_1 = 0; // Variable to store the sum of errors of PID 1
    double eIntegral_2 = 0; // Variable to store the sum of errors of PID 2
    double ePrevious1 = 0; // Variable to store the last error of PID 1
    double ePrevious2 = 0; // Variable to store the last error of PID 2

    double weightPID_1 = 0;
    double weightPID_2 = 0;
};
#endif //doublePID
