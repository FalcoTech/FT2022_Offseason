// Copyright c FIRST and other WPILib contributors.-
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include "ctre/Phoenix.h"
#include <rev/CANEncoder.h>
#include <rev/CANSparkMax.h>
// #include <rev/ColorSensorV3.h>

#include <fmt/core.h>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>
#include <frc/PS4Controller.h>
#include <frc/XboxController.h>
#include <frc/TimedRobot.h>
#include <frc/Timer.h>
#include <frc/fmt/Units.h>
#include <frc/filter/SlewRateLimiter.h>

#include <frc/drive/DifferentialDrive.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/controller/ElevatorFeedforward.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/BangBangController.h>
#include <frc/DigitalInput.h>
#include <frc/encoder.h>

#include <frc/Compressor.h>
#include <frc/DoubleSolenoid.h>

#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"
#include "./wpi/span.h"
#include <cameraserver/CameraServer.h>


#include <iostream>
#include <thread>
#include <math.h>


using namespace std;
using namespace frc;
using namespace frc2;
using namespace rev;


//General 
Compressor _compressor(frc::PneumaticsModuleType::REVPH);


//Drive Base
static const int m_leftLeadID = 41, m_rightLeadID = 40, m_leftFollowID = 43, m_rightFollowID = 42;
rev::CANSparkMax m_leftLeadMotor{m_leftLeadID, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax m_rightLeadMotor{m_rightLeadID, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax m_leftFollowMotor{m_leftFollowID, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax m_rightFollowMotor{m_rightFollowID, rev::CANSparkMax::MotorType::kBrushless};

DoubleSolenoid sol_Shift(1, frc::PneumaticsModuleType::REVPH, 0, 1);
double drivekP = 6e-5, drivekI = 1e-6, drivekD = 0, drivekIz = 0, drivekFF = 0.000015, drivekMaxOutput = 1.0, drivekMinOutput = -1.0, driveMaxRPM = 5108, driveGearRatio = 8.68;
rev::SparkMaxPIDController m_leftPID = m_leftLeadMotor.GetPIDController();
rev::SparkMaxPIDController m_rightPID = m_rightLeadMotor.GetPIDController();

rev::SparkMaxRelativeEncoder m_leftDriveEncoder = m_leftLeadMotor.GetEncoder();
rev::SparkMaxRelativeEncoder m_rightDriveEncoder = m_rightLeadMotor.GetEncoder();

frc::DifferentialDrive m_drive{m_leftLeadMotor, m_rightLeadMotor};

frc2::PIDController driveControl{drivekP, drivekI, drivekD};
frc::SlewRateLimiter<units::scalar> m_speedLimiter{3 / 1_s};


//shooter
WPI_TalonFX m_shooterMotorL{11};
WPI_TalonFX m_shooterMotorR{10};

//Intake, Ball runs
static const int m_intakeFrontID = 21, m_intakeBackID = 20;
rev::CANSparkMax m_intakeFrontMotor{m_intakeFrontID, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax m_intakeBackMotor{m_intakeBackID, rev::CANSparkMax::MotorType::kBrushless};
rev::SparkMaxRelativeEncoder m_intakeFrontEncoder = m_intakeFrontMotor.GetEncoder();
rev::SparkMaxRelativeEncoder m_intakeBackEncoder = m_intakeBackMotor.GetEncoder();
DoubleSolenoid sol_Intake(1, frc::PneumaticsModuleType::REVPH, 4, 5);


//Controllers
XboxController  *Pilot = new XboxController(0);
XboxController *CoPilot= new XboxController(1);
double joy_lStick_Y_deadband = 0.05, joy_rStick_Y_deadband = 0.05, joy_rStick_X_deadband = 0.05;

frc::Timer m_timer;


//Functions
void Robot::DriveRobotForward(units::second_t time){
  //Probably should have gear shift check in here
  m_drive.TankDrive(0.5, 0.5);
  Wait(time);
  m_drive.TankDrive(0, 0);
}

void Robot::DefaultAuto(){
 LowGear();
 ExtendIntake();
 Wait(.3_s); // .3s 

 RunIntake(true);
 Wait(0.2_s); // .5s
 StopIntake();

 RunShooter(.585);
 Wait(1_s); //1.5s
 RunIntake();
 Wait(3.5_s); //5s
 RunShooter(.7);

 m_drive.TankDrive(-0.5,-0.5); //drives forwards + intake
 Wait(2.75_s); //7s
 m_drive.TankDrive(0.5, 0.5);
 Wait(0.7s);

 m_drive.TankDrive(0,0);
 StopIntake();
 StopShooter();

 Wait(7_s);
}

//RobotInit: runs once when robot first initializes
void Robot::RobotInit(){
  m_led.SetLength(kLength);
  m_led.SetData(m_ledBuffer);
  m_led.Start();
  for (int i = 0; i < kLength; i++) {
   m_ledBuffer[i].SetRGB(255,0,0);
  }
  m_led.SetData(m_ledBuffer);
  
  m_leftFollowMotor.Follow(m_leftLeadMotor, false);
  // m_leftLeadMotor.SetInverted(true);
  m_rightFollowMotor.Follow(m_rightLeadMotor, false);
  m_rightLeadMotor.SetInverted(true);
  
  m_leftLeadMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
  m_leftFollowMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
  m_rightLeadMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
  m_rightFollowMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
  

  m_leftPID.SetP(drivekP);
  m_leftPID.SetI(drivekI);
  m_leftPID.SetD(drivekD);
  m_leftPID.SetIZone(drivekIz);
  m_leftPID.SetFF(drivekFF);
  m_leftPID.SetOutputRange(drivekMinOutput, drivekMaxOutput);

  m_rightPID.SetP(drivekP);
  m_rightPID.SetI(drivekI);
  m_rightPID.SetD(drivekD);
  m_rightPID.SetIZone(drivekIz);
  m_rightPID.SetFF(drivekFF);
  m_rightPID.SetOutputRange(drivekMinOutput, drivekMaxOutput);

  m_rightDriveEncoder.SetPosition(0.0);
  m_leftDriveEncoder.SetPosition(0.0);

  m_intakeBackMotor.SetInverted(false);
  m_intakeFrontMotor.SetInverted(true);
  
  // m_rightLiftMotor.Follow(m_leftLiftMotor);
  m_leftLiftMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_rightLiftMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_leftLiftMotor.SetInverted(true);
  m_rightLiftMotor.SetInverted(false);
  
  m_leftLiftEncoder.SetPositionConversionFactor( M_PI * 0.75 / 9);
  m_rightLiftEncoder.SetPositionConversionFactor( M_PI * 0.75 / 9);


  m_leftLiftPIDController.SetP(kLiftP);
  m_leftLiftPIDController.SetI(kLiftI);
  m_leftLiftPIDController.SetD(kLiftD);
  m_leftLiftPIDController.SetIZone(kLiftIz);
  m_leftLiftPIDController.SetFF(kLiftFF);
  m_leftLiftPIDController.SetOutputRange(kLiftMinOutput, kLiftMaxOutput);

  m_rightLiftPIDController.SetP(kLiftP);
  m_rightLiftPIDController.SetI(kLiftI);
  m_rightLiftPIDController.SetD(kLiftD);
  m_rightLiftPIDController.SetIZone(kLiftIz);
  m_rightLiftPIDController.SetFF(kLiftFF);
  m_rightLiftPIDController.SetOutputRange(kLiftMinOutput, kLiftMaxOutput);

  _gyro.SetFusedHeading(0);

  _orchestra.AddInstrument(m_shooterMotorL);
  _orchestra.AddInstrument(m_shooterMotorR);

  m_shooterMotorL.SetInverted(false);
  m_shooterMotorR.SetInverted(true);
  
  m_shooterMotorL.Config_kF(0, 0.3, 10);
  m_shooterMotorL.Config_kP(0, 0.1, 10);
  m_shooterMotorL.Config_kI(0, 0.0, 10);
  m_shooterMotorL.Config_kD(0, 0.0, 10);
  m_shooterMotorL.ConfigNominalOutputForward(0, 10);
  m_shooterMotorL.ConfigNominalOutputReverse(0, 10);
  m_shooterMotorL.ConfigPeakOutputForward(1, 10);
  m_shooterMotorL.ConfigPeakOutputReverse(0, 10);

  m_shooterMotorR.Config_kF(0, 0.3, 10);
  m_shooterMotorR.Config_kP(0, 0.1, 10);
  m_shooterMotorR.Config_kI(0, 0.0, 10);
  m_shooterMotorR.Config_kD(0, 0.0, 10);
  m_shooterMotorR.ConfigNominalOutputForward(0, 10);
  m_shooterMotorR.ConfigNominalOutputReverse(0, 10);
  m_shooterMotorR.ConfigPeakOutputForward(1, 10);
  m_shooterMotorR.ConfigPeakOutputReverse(0, 10);

  CameraServer::StartAutomaticCapture();
  
//   m_chooser.SetDefaultOption("Standard Auto", "standard");
//   m_chooser.AddOption("Only Drive","drive");
//   SmartDashboard::PutData(&m_chooser);
//   m_chooser2.SetDefaultOption("Stevie Drive", "stevie");
//   m_chooser2.AddOption("Gavin Drive", "gavin");
//   SmartDashboard::PutData(&m_chooser2);
}


//RobotPeriodic: Loops while robot is on
void Robot::RobotPeriodic() {

}

//AutonomousInit: Runs once when autonomous is initialized 
void Robot::AutonomousInit() {
  m_leftDriveEncoder.SetPosition(0);
  m_rightDriveEncoder.SetPosition(0);
  //autonState = 0;
  _gyro.SetFusedHeading(0);
  m_leftLeadMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_rightLeadMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_rightFollowMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_leftFollowMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_timer.Reset();
    m_timer.Start();
  m_drive.SetSafetyEnabled(false);
}

//AutoPeriodic: Loops while autonomous is active
void Robot::AutonomousPeriodic() {
  string selected_auto = m_chooser.GetSelected();
  // if (selected_auto == "standard"){
    DefaultAuto();
  // }
  // else if (selected_auto == "drive"){
  //  OnlyDriveAuto();
//  }
}

//TeleopInit: Runs once when teleop is initialized
void Robot::TeleopInit() {
  m_drive.SetSafetyEnabled(true);
}

//TeleopPeriodic: Loops while teleop is active. 
void Robot::TeleopPeriodic(){
  //Example of function utilization with conditional statements: 
  if (CoPilot->GetPOV() == 90){
    RetractClimber();
  }
  else if (CoPilot->GetPOV() == 270){
    ExtendClimber();
  }
  
  double leftLift = CoPilot->GetLeftY();
  double rightLift = CoPilot->GetRightY();

  m_leftLiftMotor.Set(rightLift);
  m_rightLiftMotor.Set(-1*leftLift);
  //
  //
  //
  if (CoPilot->GetRightTriggerAxis()){  
    m_intakeFrontMotor.Set(CoPilot->GetRightTriggerAxis());
    m_intakeBackMotor.Set(CoPilot->GetRightTriggerAxis());
  }
  else if (CoPilot->GetLeftTriggerAxis()){
    m_intakeFrontMotor.Set(-(CoPilot->GetLeftTriggerAxis()));
    m_intakeBackMotor.Set(-(CoPilot->GetLeftTriggerAxis()));
    
  }
  else {
    m_intakeFrontMotor.Set(0);
    m_intakeBackMotor.Set(0);
  }
    
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}


#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
