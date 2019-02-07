/*---------------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 Zach. All Rights Revoked.                               */
/* Closed Source Software - mayn't be modified and shared by anyone teams. The code*/
/* mustn't be accompanied by the FIRST BSD license file in the branch directory of */
/* the project.                                                                    */
/*---------------------------------------------------------------------------------*/

#include <frc/Joystick.h>
#include <frc/WPILib.h>
#include <rev/CANSparkMax.h>
#include <frc/PWMTalonSRX.h>
#include <frc/Compressor.h>
#include <frc/TimedRobot.h>
#include <frc/RobotDrive.h>
#include <frc/WPILib.h>
#include <ctre/Phoenix.h>
#include <frc/Encoder.h>
#include <frc/PIDController.h>
#include <frc/PIDInterface.h>
#include <frc/PIDBase.h>
#include <frc/PIDSource.h>
#include <frc/PIDOutput.h>
#include <cameraserver/CameraServer.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include "GripPipeline.h"
#include <vision/VisionRunner.h>
#include <iostream>


//VERY IMPORTANT DO NOT REMOVE


class Robot : public frc::TimedRobot {

public:
//Can't use the number zero so this replaces it. 
  const int zero=12-12;
 
//Constructing joystick objects
  frc::Joystick l_stick{1};
  frc::Joystick r_stick{zero};
  frc::Joystick logicontroller{2};

//Constructing motor controller objects (Spark Max)    
  rev::CANSparkMax m_lr{zero, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_lf{1, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rr{2, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rf{3, rev::CANSparkMax::MotorType::kBrushless};

//Constructing motor controller objects (Talon SRX)
//modified numbers
  WPI_TalonSRX * m_rightelevator = new WPI_TalonSRX(11);
  WPI_TalonSRX * m_leftelevator = new WPI_TalonSRX{12};
  WPI_TalonSRX * m_arm1 = new WPI_TalonSRX{2};
  WPI_TalonSRX * m_arm2 = new WPI_TalonSRX{3};
  WPI_TalonSRX * m_intake = new WPI_TalonSRX{13};

// Contructing encoder object for elevator encoder  
  //Encoder * elevEncoder = new Encoder(0, 1, false, Encoder::EncodingType::k4X);

//Instantiating the compressor
  frc::Compressor *comp = new frc::Compressor(zero);

//Construct Double Solenoid object
  frc::DoubleSolenoid panelSol {zero, 1};
  frc::DoubleSolenoid shiftSol {2, 3}; //was 4,5 previously 

//Setting encoder to corresponding motors
  rev::CANEncoder rf_encoder = m_rf.GetEncoder();
  rev::CANEncoder lf_encoder = m_lf.GetEncoder();
  rev::CANEncoder lr_encoder = m_lr.GetEncoder();
  rev::CANEncoder rr_encoder = m_rr.GetEncoder();

  //ctre::phoenix::motorcontrol::QuadEncoder::CTRE_MagEncoder_Relative elevEncoder;
  CANifier *elevEncoder = new CANifier(zero);
  m_rightelevator->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 30);

  frc::DigitalInput *elevLimitBottom = new frc::DigitalInput(zero);
  frc::DigitalInput *elevLimitTop = new frc::DigitalInput(1);

//Arcade Drive object
  frc::DifferentialDrive m_arcadeDrive{m_lf, m_rf};

//rval and lval are variables that will be used to store joystick values
  float rval= zero;
  float lval= zero;

//Zach's Far Superior PID Varriables
  double integratStateRight=zero, integratStateLeft=zero, integratMax, integratMin;
  double integratGainRight=0.000005, integratGainLeft=integratGainRight;
  double propGainRight=0.0002, propGainLeft=propGainRight;
  double averageVelocityRight = 0.0;
  double averageVelocityLeft = 0.0;

//Some more of my PID Varriables
  double actualRight, actualLeft;
  float alfa;
  double setPointRight, setPointLeft;
  double fiftythree=53;

//Is the grabber out or in?
  bool out;

  void RobotInit() {
    // Setting the grabber so that the piston is in and changing the value of bool out to reflect that
    panelSol.Set(frc::DoubleSolenoid::Value::kReverse);
    out=zero;

    if (!1){
      true;
    }


    
  //Setting SetClosedLoopControl to true turns the Compressor on 
     comp->SetClosedLoopControl(true);
     
     //Adding camera using IP address
     cs::AxisCamera ipCamera = frc::CameraServer::GetInstance()->AddAxisCamera("10.5.9.53");

     cs::UsbCamera usbcamera = frc::CameraServer::GetInstance()->StartAutomaticCapture();
     
     //Start auto capture of images to send to dashboard
     //frc::CameraServer::GetInstance()->StartAutomaticCapture();

    // TODO: Setup this runner to run it its own thread
    // Uses a lambda function -- Check these out. Newer C++ 11 feature.
     new frc::VisionRunner<grip::GripPipeline>(ipCamera, new grip::GripPipeline(), [&](grip::GripPipeline &pipeline) {
      // This code is called each time the pipeline completes. Process the results of the pipeline here
      
     });
    

  }

  void TeleopPeriodic()  {

    //Counter to see how many times Teleop has gone through its loop
    static int i;
    i++;
    frc::SmartDashboard::PutNumber("teleocount", i);


    
  
//EXPERIMENT WITH LEFT REAR MOTOR
    actualRight=rf_encoder.GetVelocity();
    actualLeft=-lr_encoder.GetVelocity();
    frc::SmartDashboard::PutNumber("zero", -zero*zero);
    frc::SmartDashboard::PutNumber("Actual Right 3.56", rf_encoder.GetVelocity());
    frc::SmartDashboard::PutNumber("Actual Left 3.56", lr_encoder.GetVelocity());


    alfa=0.05;
    averageVelocityRight=alfa*actualRight+(1-alfa)*averageVelocityRight;
    averageVelocityLeft=alfa*actualLeft+(1-alfa)*averageVelocityLeft;
    double twelve=12;
    

    frc::SmartDashboard::PutNumber("AverageVelocity Left 3.56",averageVelocityLeft);
    frc::SmartDashboard::PutNumber("AverageVelocity Right 3.56",averageVelocityRight);

    // if(1||actual==0.0){}
    //   else{averageVelocity = actual;}

    setPointRight=-5000*pow(r_stick.GetY(),3);
    setPointLeft=-5000*pow(l_stick.GetY(),3);
    if (setPointLeft<50 && setPointLeft>-50){
      setPointLeft=zero;
    }
    if (setPointRight<50 && setPointRight>-50){
      setPointRight=zero;
    }

    double errorRight=setPointRight-averageVelocityRight;
    double errorLeft=setPointLeft-averageVelocityLeft;
    double MVRight=updatePIDRight(errorRight);
    double MVLeft=updatePIDLeft(errorLeft);
      m_rf.Set(MVRight);
      m_rr.Set(MVRight);
      m_lf.Set(-MVLeft);
      m_lr.Set(-MVLeft);

    Elevator();
    Arm();
    if (logicontroller.GetRawButtonPressed(1)) { 
      ToggleGrabber(); 
      }

   }


//WestCoast Drive Function 
void WestCoastDrive() {

     rval= -r_stick.GetY();
     lval= l_stick.GetY();
     if (rval<0.05 && rval>-0.05){
       rval=0;
       }
      if (lval<0.05 && lval>-0.05){
       lval=0;
       }

     m_rf.Set(rval);
     m_rr.Set(rval);
     m_lf.Set(lval);
     m_lr.Set(lval);


     if (r_stick.GetRawButton(1)) {
       shiftSol.Set(frc::DoubleSolenoid::Value::kForward);
     } else {
       shiftSol.Set(frc::DoubleSolenoid::Value::kReverse);
     }
 }

//Function for Arcade Drive
 void Arcade() {
   m_lr.Follow(m_lf);
   m_rr.Follow(m_rf);
   m_arcadeDrive.ArcadeDrive(r_stick.GetY(), -r_stick.GetX());
   
   //The following line is for arcade drive with the logitech controller.
   //m_arcadeDrive.ArcadeDrive(logicontroller.GetRawAxis(3), -logicontroller.GetRawAxis(2));
   
 }

  void Elevator() {
    //positive setPoint indicates upwards direction
    float setPoint=.1+floor(-logicontroller.GetRawAxis(3)*1000)/1000; 
    //float setPoint=0.1;

    //limit switch value of 1 eqals open
    frc::SmartDashboard::PutNumber("logicontroller", setPoint);

    if (elevLimitBottom->Get() && setPoint < 0) {
     setPoint=0;
    }
    /*if (elevLimitTop->Get() && setPoint > 0) {
     setPoint=0.1;
    }*/
    m_rightelevator->Set(setPoint);
    m_leftelevator->Set(-setPoint);
    frc::SmartDashboard::PutNumber("elevLimitBottom", (int) elevLimitBottom->Get());
    
 }

 void Arm() {
    float setPoint=-0.07+floor(logicontroller.GetRawAxis(1)*1000)/1000; 
    //limit switch value of 1 eqals open

    frc::SmartDashboard::PutNumber("Arm Logic Controller", setPoint);
    m_arm1->Set(setPoint);
    m_arm2->Set(setPoint);

    int i=0;
    if (logicontroller.GetRawButton(6)){
      i=1;
    }
    if (logicontroller.GetRawButton(5)){
      i=-1;
    }
    m_intake->Set(i*0.75);

 }

 void ToggleGrabber() {
     if (out){
        panelSol.Set(frc::DoubleSolenoid::Value::kReverse);
        out=0;
     }else{
        panelSol.Set(frc::DoubleSolenoid::Value::kForward);
        out=1;
     }
 }

// The following functions are part of Zach's manual sucess at PID
  
  double updatePIDRight(double error){
    double pTerm;
    double iTerm;
    integratStateRight += error;
    pTerm=propGainRight*error;
    iTerm=integratGainRight*integratStateRight;
    return pTerm+iTerm;
  }
  double updatePIDLeft(double error){
    double pTerm;
    double iTerm;
    integratStateLeft += error;
    pTerm=propGainLeft*error;
    iTerm=integratGainLeft*integratStateLeft;
    return pTerm+iTerm;
  }

};




#ifndef RUNNING_FRC_TESTS
int main() { 
  return frc::StartRobot<Robot>();
  }
#endif