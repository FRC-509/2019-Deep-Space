/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <frc/Joystick.h>
#include <frc/WPILib.h>
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

  float rval= 0;
  float lval= 0;
  
  // std::shared_ptr<NetworkTable> visiontable;
  // visionTable = Network::GetTable("GRIP/myContoursReport");
  nt::NetworkTableEntry xEntry;
  nt::NetworkTableEntry yEntry;

  void RobotInit() {
    leftencoder.SetDistancePerPulse(1);
    rightencoder.SetDistancePerPulse(1);
    frc::CameraServer::GetInstance()->StartAutomaticCapture();
    frc::CameraServer::GetInstance()->AddAxisCamera("10.5.9.53");
    //frc::VisionRunner<cs::AxisCamera>::VisionRunner(frc::CameraServer::AddAxisCamera("10.5.9.53"), grip::GripPipeline(), VisionAlert());
    
    comp->SetClosedLoopControl(true);

    auto inst = nt::NetworkTableInstance::GetDefault();
    auto table = inst.GetTable("datatable");
    inst.StartClientTeam(509);
    xEntry = table->GetEntry("X");
    yEntry = table->GetEntry("Y");
    double x = 0;
    double y = 0;

    xEntry.SetDouble(x);
    yEntry.SetDouble(y);
    
    //nt::AddEntryListener("X", table, key, entry, value, flags);
  }
  void TeleopPeriodic()  {
  
    comp->SetClosedLoopControl(true);

    //TankDrive();

    if (l_stick.GetRawButton(2)) {
      panelsol.Set(frc::DoubleSolenoid::Value::kForward);
    } else {
      panelsol.Set(frc::DoubleSolenoid::Value::kReverse);
    }

    // PID(-2000,-2000,0.000038,0,0);
    // motorSet(LSet, RSet);

    frc::SmartDashboard::PutNumber("Encoder Left", leftencoder.GetRate());
    frc::SmartDashboard::PutNumber("Encoder Right", rightencoder.GetRate());

      
  }

  void TankDrive() {
      rval= -r_stick.GetY();
      lval= l_stick.GetY();

      m_rf->Set(rval);
      m_rr->Set(rval);
      m_lf->Set(lval);
      m_lr->Set(lval);
  }
  float PIDCalc(float expected, float actual, float Kp, float Ki, float Kd){
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
      RateLeft=leftencoder.GetRate();
      RateRight=-rightencoder.GetRate();
      LSet=PIDCalc(-LSpeed, RateLeft,Kp,Ki,Kd);
      RSet=PIDCalc(RSpeed, RateRight,Kp,Ki,Kd);
    }


  }

 private:
  float errorP=0; float errorInt=0; float errorDeriv=0;
  float errorLast=0; float output=0;
  float RateLeft=0; float RateRight=0;
  float LSet=0; float RSet=0;
  bool boolPID=true;
  
  frc::Joystick l_stick{0};
  frc::Joystick r_stick{1};
  frc::PWMTalonSRX m_motor{0};
  frc::Compressor *comp = new frc::Compressor(0);
  WPI_TalonSRX * m_rf = new WPI_TalonSRX (0);
  WPI_TalonSRX * m_rr = new WPI_TalonSRX (1);
  WPI_TalonSRX * m_lf = new WPI_TalonSRX (2);
  WPI_TalonSRX * m_lr = new WPI_TalonSRX (3);
  frc::Encoder leftencoder {2, 3};
  frc::Encoder rightencoder {0, 1,true};
  frc::DoubleSolenoid panelsol {4, 5};
  
  //frc::PIDController leftcontrol(0,0,0,leftencoder.GetRate(),l_stick.GetY());

};

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif