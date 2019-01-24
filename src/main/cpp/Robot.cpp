/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

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


class Robot : public frc::TimedRobot {

public:

//Constructing joystick objects
 frc::Joystick r_stick{0};
 frc::Joystick l_stick{1};
 frc::Joystick logicontroller{2};

//Constructing motor controller objects (Spark Max)    
 rev::CANSparkMax m_lr{0, rev::CANSparkMax::MotorType::kBrushless};
 rev::CANSparkMax m_lf{1, rev::CANSparkMax::MotorType::kBrushless};
 rev::CANSparkMax m_rr{2, rev::CANSparkMax::MotorType::kBrushless};
 rev::CANSparkMax m_rf{3, rev::CANSparkMax::MotorType::kBrushless};

//Constructing motor controller objects (Talon SRX)
 WPI_TalonSRX * m_rightelevator = new WPI_TalonSRX{4};
 WPI_TalonSRX * m_leftelevator = new WPI_TalonSRX{5};

//Instantiating the compressor
 frc::Compressor *comp = new frc::Compressor(0);

//Construct Double Solenoid object
 frc::DoubleSolenoid GearSol {2, 3};

//Setting encoder to corresponding motors
 rev::CANEncoder rf_encoder = m_rf.GetEncoder();
 rev::CANEncoder lf_encoder = m_lf.GetEncoder();

 //PIDController for the right front and left front motors
 rev::CANPIDController rf_pidController = m_rf.GetPIDController();
 rev::CANPIDController rr_pidController = m_rr.GetPIDController();

 //frc::DigitalInput climberlimit = new frc::DigitalInput(6);

//Arcade Drive object
 frc::DifferentialDrive m_arcadeDrive{m_lf, m_rf};

//rval and lval are variables that will be used to store joystick values
 float rval= 0;
 float lval= 0;

 //PID variables
 double kP = 0.1, kI = 1e-4, kD = 1, kIz = 0, kFF = 0, kMaxOutput = 1, kMinOutput = -1;
 const double MaxRPM = 5676;



  void RobotInit() {
     //Setting SetClosedLoopControl to true turns the Compressor on 
     comp->SetClosedLoopControl(true);
     
     //Adding camera using IP address
     cs::AxisCamera ipCamera = frc::CameraServer::GetInstance()->AddAxisCamera("10.5.9.53");
     
     //Start auto capture of images to send to dashboard
     frc::CameraServer::GetInstance()->StartAutomaticCapture();

    // TODO: Setup this runner to run it its own thread
    // Uses a lambda function -- Check these out. Newer C++ 11 feature.
     new frc::VisionRunner<grip::GripPipeline>(ipCamera, new grip::GripPipeline(), [&](grip::GripPipeline &pipeline) {
      // This code is called each time the pipeline completes. Process the results of the pipeline here
      
     });
    
    //Set PID coefficients
     rf_pidController.SetP(kP);
     rf_pidController.SetI(kI);
     rf_pidController.SetD(kD);
     rf_pidController.SetIZone(kIz);
     rf_pidController.SetFF(kFF);
     rf_pidController.SetOutputRange(kMinOutput, kMaxOutput);
     
     rr_pidController.SetP(kP);
     rr_pidController.SetI(kI);
     rr_pidController.SetD(kD);
     rr_pidController.SetIZone(kIz);
     rr_pidController.SetFF(kFF);
     rr_pidController.SetOutputRange(kMinOutput, kMaxOutput);
    

     //Display PID coefficients on SmartDashboard
     frc::SmartDashboard::PutNumber("P Gain", kP);
     frc::SmartDashboard::PutNumber("I Gain", kI);
     frc::SmartDashboard::PutNumber("D Gain", kD);
     frc::SmartDashboard::PutNumber("I Zone", kIz);
     frc::SmartDashboard::PutNumber("Feed Forward", kFF);
     frc::SmartDashboard::PutNumber("Max Output", kMaxOutput);
     frc::SmartDashboard::PutNumber("Min Output", kMinOutput);
  }

  void TeleopPeriodic()  {

    // Setting PID coeffiencients for Zach's manual program
    // PID(-2000,-2000,0.000038,0,0);
    // motorSet(LSet, RSet);

     WestCoastDrive();
     //Arcade();
     //GoDistance(20.0);

     // PID coefficients are outputted to SmartDashboard
     double p = frc::SmartDashboard::GetNumber("P Gain", 0);
     double i = frc::SmartDashboard::GetNumber("I Gain", 0);
     double d = frc::SmartDashboard::GetNumber("D Gain", 0);
     double iz = frc::SmartDashboard::GetNumber("I Zone", 0);
     double ff = frc::SmartDashboard::GetNumber("Feed Forward", 0);
     double max = frc::SmartDashboard::GetNumber("Max Output", 0);
     double min = frc::SmartDashboard::GetNumber("Min Output", 0);

     // if PID coefficients on SmartDashboard have changed, write new values to controller
     if((p != kP)) { rf_pidController.SetP(p); kP = p; }
     if((i != kI)) { rf_pidController.SetI(i); kI = i; }
     if((d != kD)) { rf_pidController.SetD(d); kD = d; }
     if((iz != kIz)) { rf_pidController.SetIZone(iz); kIz = iz; }
     if((ff != kFF)) { rf_pidController.SetFF(ff); kFF = ff; }
     if((max != kMaxOutput) || (min != kMinOutput)) {
       rf_pidController.SetOutputRange(min, max);
       kMinOutput = min; kMaxOutput = max;
     }
    

    // More code for PID Testing
    //  double SetPoint = 5;
    //  rf_pidController.SetReference(SetPoint, rev::ControlType::kPosition);
    //  rr_pidController.SetReference(SetPoint, rev::ControlType::kPosition);

    //  frc::SmartDashboard::PutNumber("SetPoint", SetPoint);
     frc::SmartDashboard::PutNumber("Right Encoder", rf_encoder.GetPosition());
     frc::SmartDashboard::PutNumber("Left Encoder", -lf_encoder.GetPosition());

   }


//WestCoast Drive Function 
void WestCoastDrive() {

     rval= -r_stick.GetY();
     lval= l_stick.GetY();

     m_rf.Set(rval);
     m_rr.Set(rval);
     m_lf.Set(lval);
     m_lr.Set(lval);

     if (l_stick.GetRawButton(2)) {
       GearSol.Set(frc::DoubleSolenoid::Value::kForward);
     } else {
       GearSol.Set(frc::DoubleSolenoid::Value::kReverse);
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

//Function for Elevator
 void Elevator() {
   m_rightelevator->Set(logicontroller.GetRawAxis(3));
   m_leftelevator->Set(-logicontroller.GetRawAxis(3));
  //  while (climberlimit.Get()) {
  //   m_rightelevator->Set(0);
  //   m_leftelevator->Set(0);
  //  }
 }

// The following functions are part of Zach's manual attempt at PID
  /*float PIDCalc(float expected, float actual, float Kp, float Ki, float Kd){
    errorP=expected-actual;
    errorDeriv=errorP-errorLast;
    errorInt=errorP+errorInt;
    output=Kp*errorP+Ki*errorInt+Kd*errorDeriv;
    return output;
  }
  void motorSet(float LSet, float RSet){
    m_lf->Set(LSet); 
    m_lr->Set(LSet);
    m_rf->Set(RSet); 
    m_rr->Set(RSet);
  }
  void PID(float LSpeed, float RSpeed, float Kp, float Ki, float Kd){
    if(boolPID=true){
      RateLeft=lf_encoder.GetRate();
      RateRight=-rf_encoder.GetRate();
      LSet=PIDCalc(-LSpeed, RateLeft,Kp,Ki,Kd);
      RSet=PIDCalc(RSpeed, RateRight,Kp,Ki,Kd);
    }*/


  };


#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif