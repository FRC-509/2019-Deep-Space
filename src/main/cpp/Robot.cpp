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
#include <cameraserver/CameraServer.h>
#include <vision/VisionRunner.h>
#include <iostream>

class Robot : public frc::TimedRobot {

public:
//Can't use the number zero so this replaces it. 
  #define COMP_ROBOT
  #ifdef COMP_ROBOT
    #define RIGHT_STICK 0
    #define LEFT_STICK 1
    #define CONTROLLER 2

    //Constructing motor controller objects (Spark Max)
    #define SPARK_MAX_LEFT_REAR 4
    #define SPARK_MAX_LEFT_FRONT 1
    #define SPARK_MAX_RIGHT_REAR 2
    #define SPARK_MAX_RIGHT_FRONT 3

    #define TALON_SRX_ELEVATOR_RIGHT 12
    #define TALON_SRX_ELEVATOR_LEFT 13
    #define TALON_SRX_ARM_1 4
    #define TALON_SRX_ARM_2 5
    #define TALON_SRX_INTAKE 14
    #define TALON_SRX_CLIMBER_1 15
  
  #else
    #define RIGHT_STICK 0
    #define LEFT_STICK 1
    #define CONTROLLER 2

    //Constructing motor controller objects (Spark Max)
    #define SPARK_MAX_LEFT_REAR 0
    #define SPARK_MAX_LEFT_FRONT 1
    #define SPARK_MAX_RIGHT_REAR 2
    #define SPARK_MAX_RIGHT_FRONT 3

    #define TALON_SRX_ELEVATOR_RIGHT 11
    #define TALON_SRX_ELEVATOR_LEFT 12
    #define TALON_SRX_ARM_1 2
    #define TALON_SRX_ARM_2 3
    #define TALON_SRX_INTAKE 13
    #define TALON_SRX_CLIMBER_1 4
    //#define TALON_SRX_CLIMBER_2 5

  #endif

//Constructing joystick objects
  
  frc::Joystick r_stick{ RIGHT_STICK };
  frc::Joystick l_stick{ LEFT_STICK };
  frc::Joystick logicontroller{ CONTROLLER };

//Constructing motor controller objects (Spark Max)

  rev::CANSparkMax m_lr{ SPARK_MAX_LEFT_REAR , rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_lf{ SPARK_MAX_LEFT_FRONT , rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rr{ SPARK_MAX_RIGHT_REAR , rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rf{ SPARK_MAX_RIGHT_FRONT , rev::CANSparkMax::MotorType::kBrushless};

//Constructing motor controller objects (Talon SRX)
//modified numbers

#define ELEV_HEIGHT_LOW 5000
#define ELEV_HEIGHT_MEDIUM 10500
#define ELEV_HEIGHT_HIGH 13000

//Sets heights in encoder counts that need to be reached for each slot on the rocket. 
#define ELEV_SPEED 250
#define DISK_ONE_HEIGHT 830
#define DISK_TWO_HEIGHT 13480
#define DISK_THREE_HEIGHT 22000
#define BALL_DISPLACEMENT 5680

bool isBall=0;

//Sets climber values
#define CLIMBER_SPEED 0.5

  WPI_TalonSRX * m_rightelevator = new WPI_TalonSRX( TALON_SRX_ELEVATOR_RIGHT );
  WPI_TalonSRX * m_leftelevator = new WPI_TalonSRX{ TALON_SRX_ELEVATOR_LEFT };
  WPI_TalonSRX * m_arm1 = new WPI_TalonSRX{ TALON_SRX_ARM_1 };
  WPI_TalonSRX * m_arm2 = new WPI_TalonSRX{ TALON_SRX_ARM_2 };
  WPI_TalonSRX * m_intake = new WPI_TalonSRX{ TALON_SRX_INTAKE };
  WPI_TalonSRX * m_climber_1 = new WPI_TalonSRX{ TALON_SRX_CLIMBER_1 };

// Contructing encoder object for elevator encoder  
  //Encoder * elevEncoder = new Encoder(0, 1, false, Encoder::EncodingType::k4X);

//Instantiating the compressor
  frc::Compressor *comp = new frc::Compressor(0);

//Construct Double Solenoid object
  frc::DoubleSolenoid panelSol {0, 1};
  frc::DoubleSolenoid shiftSol {2, 3}; //was 4,5 previously 

//Setting encoder to corresponding motors
  rev::CANEncoder rf_encoder = m_rf.GetEncoder();
  rev::CANEncoder lf_encoder = m_lf.GetEncoder();
  rev::CANEncoder lr_encoder = m_lr.GetEncoder();
  rev::CANEncoder rr_encoder = m_rr.GetEncoder();

  //ctre::phoenix::motorcontrol::QuadEncoder::CTRE_MagEncoder_Relative elevEncoder;
  // CANifier *canElevEncoder = new CANifier(0);
  int canDisplayCount = 0;
  int canInit = 0; 

  frc::DigitalInput *elevLimitBottom = new frc::DigitalInput(0);
  frc::DigitalInput *elevLimitTop = new frc::DigitalInput(1);

//Arcade Drive object
  frc::DifferentialDrive m_arcadeDrive{m_lf, m_rf};

//rval and lval are variables that will be used to store joystick values
  float rval= 0;
  float lval= 0;

//Zach's Far Superior PID Varriables
  double integratStateRight=0, integratStateLeft=0, integratStateElev=0;
  double integratGainRight=0.000005, integratGainLeft=integratGainRight, integratGainElev=0.000001;//0.0000001;
  double propGainRight=0.0002, propGainLeft=propGainRight, propGainElev=0.00038;
  double averageVelocityRight = 0.0;
  double averageVelocityLeft = 0.0;
  int currentElevLevel=1;

//Some more of my PID Varriables
  double actualRight, actualLeft;
  float alfa;
  double setPointRight, setPointLeft, setPointElev;
  double setSetPointElev;
  double fiftythree=53;
  int itwoc = 0;
  bool climbActivated=false;
  float leftDistInitial;

//Is the grabber out or in?
  bool out;

//String display what mode the robot is currently in. As of right not, does not work.
  std::string gameMode;

  void RobotInit() {
    
    elevEncoderInit();

     //Setting SetClosedLoopControl to true turns the Compressor on 
     comp->SetClosedLoopControl(true);
     
     //Adding camera using IP address
     cs::AxisCamera ipCamera = frc::CameraServer::GetInstance()->AddAxisCamera("10.5.9.53");
     cs::UsbCamera usbcamera = frc::CameraServer::GetInstance()->StartAutomaticCapture();
  }

  void elevEncoderInit() {
    /* nonzero to block the config until success, zero to skip checking */
    const int kTimeoutMs = 30;
    //setPointElev = 5000;
    
    /* Configure sensor on talon to check CANifier */
    if (m_rightelevator->ConfigSelectedFeedbackSensor(
      FeedbackDevice::CTRE_MagEncoder_Relative, 0, kTimeoutMs)) {
      canInit |= 0x1 << 0;
    }

    /* Set sensor positions to some known position */
    if (m_rightelevator->SetSelectedSensorPosition(51, 0, kTimeoutMs)) {
      canInit |= 0x1 << 1;
    }

    frc::SmartDashboard::PutNumber("canInit:", canInit);
    //m_rightelevator->SetSelectedSensorPosition(0, 0, 50); 
  }

  void AutonomousInit() {
    // Setting the grabber so that the piston is out
    panelSol.Set(frc::DoubleSolenoid::Value::kForward);
    //changing the value of bool out to reflect that
    out=1;
  }
  
  void AutonomousPeriodic() {
    TeleopPeriodic();
  }

  void TeleopInit() {
    // Setting the grabber so that the piston is out
    panelSol.Set(frc::DoubleSolenoid::Value::kForward);
    //changing the value of bool out to reflect that
    out=1;
    setSetPointElev= m_rightelevator->GetSelectedSensorPosition(0);
    setPointElev= m_rightelevator->GetSelectedSensorPosition(0);
  }

  void TeleopPeriodic()  {


    //Counter to see how many times Teleop has gone through its loop
    static int i;
    i++;
    frc::SmartDashboard::PutNumber("teleocount", i);


    

    actualRight=rf_encoder.GetVelocity();
    actualLeft=-lr_encoder.GetVelocity();
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
      setPointLeft=0;
    }
    if (setPointRight<50 && setPointRight>-50){
      setPointRight=0;
    }

    #define DRIVE_PID
    #ifdef DRIVE_PID

    double errorRight=setPointRight-averageVelocityRight;
    double errorLeft=setPointLeft-averageVelocityLeft;
    double MVRight=updatePIDRight(errorRight);
    double MVLeft=updatePIDLeft(errorLeft);

    m_rf.Set(MVRight);
    m_rr.Set(MVRight);
    m_lf.Set(-MVLeft);
    m_lr.Set(-MVLeft);

    #else
    WestCoastDrive();
    #endif

    // If "A" pressed, set elev to medium height
    // if (logicontroller.GetRawButtonPressed(4)) {
    //   if (currentElevLevel=1){
    //     setPointElev = DISK_TWO_HEIGHT;
    //     currentElevLevel=2;
    //   } else {
    //     setPointElev = DISK_THREE_HEIGHT;
    //     currentElevLevel=3;
    //   }
       
    // }

    
    
    #define ELEV_PID

    #ifdef ELEV_PID
    double errorElev = setSetPointElev - m_rightelevator->GetSelectedSensorPosition(0);
    double MVElev = updatePIDElev(errorElev);
    m_rightelevator->Set(MVElev/2);
    m_leftelevator->Set(-MVElev/2);
    if (logicontroller.GetRawButtonPressed(1)) {
         isBall = !isBall;
    } 
    if (logicontroller.GetRawButtonPressed(2)) {
         setPointElev = DISK_ONE_HEIGHT+isBall*BALL_DISPLACEMENT;
    } 
    if (logicontroller.GetRawButtonPressed(3)) {
         setPointElev = DISK_TWO_HEIGHT+isBall*BALL_DISPLACEMENT;
    } 
    if (logicontroller.GetRawButtonPressed(4)) {
         setPointElev = DISK_THREE_HEIGHT;
    } 
    frc::SmartDashboard::PutNumber("SetPointElev1", setPointElev);

    if (isBall) {
      gameMode = "Cargo";
    } else {
      gameMode = "Panel";
    }

    frc::SmartDashboard::PutString("Panel or Cargo", gameMode);
    frc::SmartDashboard::PutNumber("Panel or Cargo2", isBall);

    
    float joyVal3=-logicontroller.GetRawAxis(3);
    //setPointElev = 5000;

    if ( joyVal3 < -0.05 || joyVal3 > 0.05 ) {
      //setPointElev = 1000*joyVal3 +  m_leftelevator->GetSelectedSensorPosition(0);
      if (joyVal3>0) {
        setPointElev = 100*joyVal3 + setPointElev;
      } else {
        setPointElev = 100*pow(joyVal3, 3) + setPointElev;
      }
      
    }
    frc::SmartDashboard::PutNumber("SetPointElev2", setPointElev);

    if (setPointElev>23000){
      setPointElev=23000;
      }

    if (setPointElev<300){
      setPointElev=300;
      }
    frc::SmartDashboard::PutNumber("SetPointElev3", setPointElev);
    if (setSetPointElev != setPointElev){
      if (setSetPointElev<setPointElev+100 && setSetPointElev>setPointElev-100){
        setSetPointElev=setPointElev;
      }
      if (setPointElev>setSetPointElev){
        setSetPointElev += ELEV_SPEED;
      } else {
        setSetPointElev -= ELEV_SPEED;
      }


    }
    #else
    Elevator();
    #endif

    Arm();
    if (logicontroller.GetRawButtonPressed(10)) { 
      ToggleGrabber(); 
      }


    climber();
    
    
    if (r_stick.GetRawButton(1)) {
       shiftSol.Set(frc::DoubleSolenoid::Value::kForward);
     } else {

       shiftSol.Set(frc::DoubleSolenoid::Value::kReverse);
     }

    if(canDisplayCount++ % 20 == 0)
    {
      /* CANifier */
      //std::cout << "CANifier:\tPosition: " << _can->GetQuadraturePosition() << "\tVelocity" << _can->GetQuadratureVelocity() <<
      // frc::SmartDashboard::PutNumber("CANifier Position: ", canElevEncoder->GetQuadraturePosition());
      // frc::SmartDashboard::PutNumber("CANifier Velocity", canElevEncoder->GetQuadratureVelocity()); 

      /* TalonSRX */
      //std::endl << "Talon:\t\t\tPosition: " << _tal->GetSelectedSensorPosition(0) <<"\tVelocity" << _tal->GetSelectedSensorVelocity(0)
      frc::SmartDashboard::PutNumber("Talon Position: ", m_rightelevator->GetSelectedSensorPosition(0)); 
      frc::SmartDashboard::PutNumber("Talon Velocity", m_rightelevator->GetSelectedSensorVelocity(0));

      //frc::SmartDashboard::PutNumber("New Talon SRX Position", (double) m_leftelevator->GetSensorCollection());
    
      //m_leftelevator->Get
      /* New line to deliniate each loop */
      //< std::endl << std::endl;
    }
    /* Run talon in PercentOutput mode always */
    //_tal->Set(ControlMode::PercentOutput, _joy->GetY());
    //#define EXPLOSION_BEGIN 1
    #ifdef EXPLOSION_BEGIN
        if (logicontroller.GetRawButtonPressed(9)){
          itwoc=0;
          climbActivated=true;
        }
        if (logicontroller.GetRawButtonPressed(10)) {
          itwoc = 0;
          climbActivated = false;
        }
        if (climbActivated){
          itwoc++;
          if (itwoc < 3*50){
            m_rightelevator->Set(-0.3);
            m_leftelevator->Set(0.3);
          }
          if (itwoc>50 && itwoc < 3*50){
            m_climber_1->Set( -CLIMBER_SPEED );

          }
        }

    #endif
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
    float setPoint = .12+floor(-logicontroller.GetRawAxis(3)*1000/1)/1000;


    
    /*if ((setPoint < 0)) {
      setPoint=.1+floor(pow(-logicontroller.GetRawAxis(3),));
    } else {
      setPoint=.1+floor(-logicontroller.GetRawAxis(3));
    }*/
    
    
    //float setPoint=0.1+(-logicontroller.GetRawAxis(3));
    
    //float setPoint=0.1;

    //limit switch value of 1 eqals open
    frc::SmartDashboard::PutNumber("logicontroller", setPoint);

    if (elevLimitBottom->Get() && setPoint < 0) {
     setPoint=0.1;
    }
    /*if (elevLimitTop->Get() && setPoint > 0) {
     setPoint=0.1;
    }*/
    m_rightelevator->Set((setPoint/2000)*1000);
    m_leftelevator->Set((-setPoint/2000)*1000);
    frc::SmartDashboard::PutNumber("elevLimitBottom", (int) elevLimitBottom->Get());
    frc::SmartDashboard::PutNumber("elevLimitTop", (int) elevLimitTop->Get());
    
 }

 void Arm() {
    float setPoint=-0.05+floor(logicontroller.GetRawAxis(1)*1000)/1000; 
    //limit switch value of 1 eqals open

    frc::SmartDashboard::PutNumber("Arm Logic Controller", setPoint);
    m_arm1->Set(setPoint);
    m_arm2->Set(setPoint);

    int i=0;
    //idiot proofing the intake: If both buttons are pressed it stops the intake
    if (logicontroller.GetRawButton(5) && logicontroller.GetRawButton(6)){
      i=0;
    } else {
      if (logicontroller.GetRawButton(6)){
        i=-1;
      }
      if (logicontroller.GetRawButton(5)){
        i=1;
      }
    }
    m_intake->Set(i);

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
  double updatePIDElev(double error){
    double pTerm;
    double iTerm;
    
    pTerm=propGainElev*error;

    // if (integratStateElev>10000){
    //   integratStateElev=10000;
    // }
    // if (integratStateElev<-10000){
    //   integratStateElev=-10000;
    // }
    integratStateElev += error;
    iTerm=integratGainElev*integratStateElev;
    
    return pTerm+iTerm;
  }
  double moveADistance(float distance){
    float leftDistNow=-lr_encoder.GetPosition();

    if (leftDistNow-leftDistInitial<distance){
      setPointRight=500;
      setPointLeft=500;
    }

  }

  void climber() {
    // Set to true to turn motors on permanently 
    static bool dbgclimber = false;

    if (dbgclimber) {
      m_climber_1->Set(0.25);
    }

    
    else if (r_stick.GetRawButton(3)) {
      m_climber_1->Set( CLIMBER_SPEED );
    }
    //Sets climber values
    else if (r_stick.GetRawButton(2)) {
      m_climber_1->Set( -CLIMBER_SPEED );
    }
    //if not pressed, stop motor
    else {
      m_climber_1->Set(0);
    }
}
};




#ifndef RUNNING_FRC_TESTS
int main() { 
  return frc::StartRobot<Robot>();
  }
#endif