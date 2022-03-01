// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include "ctre/Phoenix.h"
#include <rev/CANEncoder.h>
#include <rev/CANSparkMax.h>
#include <rev/ColorSensorV3.h>
//#include "rev/CANEncoder.h"

#include <fmt/core.h>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>
#include <frc/PS4Controller.h>
#include <frc/XboxController.h>
#include <frc/TimedRobot.h>
#include <frc/Timer.h>
#include <frc/fmt/Units.h>

#include <frc/drive/DifferentialDrive.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <frc/controller/PIDController.h>
#include <frc/DigitalInput.h>
#include <frc/encoder.h>

#include <frc/Compressor.h>
#include <frc/DoubleSolenoid.h>

#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
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
double m_leftOut = 0, m_rightOut = 0;
DoubleSolenoid sol_Shift(1, frc::PneumaticsModuleType::REVPH, 0, 1);

double drivekP = 6e-5, drivekI = 1e-6, drivekD = 0, drivekIz = 0, drivekFF = 0.000015, drivekMaxOutput = 1.0, drivekMinOutput = -1.0, driveMaxRPM = 5108, driveGearRatio = 8.68;
rev::SparkMaxPIDController m_leftPID = m_leftLeadMotor.GetPIDController();
rev::SparkMaxPIDController m_rightPID = m_rightLeadMotor.GetPIDController();

rev::SparkMaxRelativeEncoder m_leftEncoder = m_leftLeadMotor.GetEncoder();
rev::SparkMaxRelativeEncoder m_rightEncoder = m_rightLeadMotor.GetEncoder();

frc::DifferentialDrive m_drive{m_leftLeadMotor, m_rightLeadMotor};
frc2::PIDController driveControl{drivekP, drivekI, drivekD};

WPI_PigeonIMU _gyro(1);
Rotation2d getGyroAngle(){
  double gyroroation = _gyro.GetFusedHeading();
  return Rotation2d(units::degree_t(gyroroation));
}

//shooter
WPI_TalonFX m_shooterMotorL{11};
WPI_TalonFX m_shooterMotorR{10};
frc2::PIDController m_shooterPID{0.0008, 0, 0, units::time::second_t(50)};
//frc2::PIDController m_shooterPID{}
double shooterGearRatio = 1;
double shooterMaxRPM = 5742/*max rpm of falcon 500 no load -10%*/ * shooterGearRatio, shooterMinRPM = 5742 * -0.1/*percent output limit for reverse*/ * shooterGearRatio;
double shooterTargetRPM = 0;

//Path Weaver
double ks, kv, ka;
Pose2d position(units::meter_t(0), units::meter_t(0), Rotation2d(units::degree_t(135)));
DifferentialDriveKinematics kinematics(units::length::meter_t(0.7633));
//DifferentialDriveOdometry odometry(getGyroAngle(), position);
frc::SimpleMotorFeedforward<units::meters> feedfwd(0.22_V, 1.98 * 1_V * 1_s / 1_m, 0.2 * 1_V * 1_s * 1_s / 1_m);

//Intake, Ball runs
static const int m_intakeFrontID = 20, m_intakeBackID = 21;
rev::CANSparkMax m_intakeFrontMotor{m_intakeFrontID, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax m_intakeBackMotor{m_intakeBackID, rev::CANSparkMax::MotorType::kBrushless};
DoubleSolenoid sol_Intake(1, frc::PneumaticsModuleType::REVPH, 2, 3);

//Lift and Climb
static const int m_leftLiftID = 31, m_rightLiftID = 30;
rev::CANSparkMax m_leftLiftMotor{m_leftLiftID, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax m_rightLiftMotor{m_rightLiftID, rev::CANSparkMax::MotorType::kBrushless};
DoubleSolenoid sol_Climber(1, frc::PneumaticsModuleType::REVPH, 4, 5);


//Controllers
XboxController  *Pilot = new XboxController(0);
XboxController *CoPilot= new XboxController(1);
double joy_lStick_Y_deadband = 0.05, joy_rStick_Y_deadband = 0.05, joy_rStick_X_deadband = 0.05;

frc::Timer m_timer;

double getLeftEncoderDist(){
  return m_leftEncoder.GetPosition() / 42/*ticks per rev*/ * driveGearRatio * (2 * M_PI * 0.0762)/*dist per rev in meters*/;
}

double getRightEncoderDist(){
  return m_rightEncoder.GetPosition() / 42 * driveGearRatio * (2 * M_PI * 0.0762);
}
/*
void move(double dist){
  double rotations = (dist/(6*M_PI))*8.68;
  //DifferentialDrive::
}
*/
void Robot::RobotInit() {
  
  m_leftFollowMotor.Follow(m_leftLeadMotor, false);
  //m_leftLeadMotor.SetInverted(true);
  m_rightFollowMotor.Follow(m_rightLeadMotor, false);
  //m_rightLeadMotor.SetInverted(true);

  m_leftLeadMotor.SetInverted(true);

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

  m_rightEncoder.SetPosition(0.0);
  m_leftEncoder.SetPosition(0.0);

  m_intakeBackMotor.SetInverted(false);
  m_intakeFrontMotor.SetInverted(false);
  
  m_rightLiftMotor.Follow(m_leftLiftMotor);
  m_leftLiftMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_rightLiftMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_leftLiftMotor.SetInverted(true);
  m_rightLiftMotor.SetInverted(false);

  _gyro.SetFusedHeading(0);

  
}

void Robot::RobotPeriodic() {
  //position = odometry->Update(getGyroAngle(), units::meter_t(getLeftEncoderDist()), units::meter_t(getRightEncoderDist()));
  //autoAimPID.SetSetpoint(0);
}

void Robot::AutonomousInit() {
  m_leftEncoder.SetPosition(0);
  m_rightEncoder.SetPosition(0);
  //autonState = 0;
  _gyro.SetFusedHeading(0);
  m_leftLeadMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_rightLeadMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_rightFollowMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_leftFollowMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_timer.Reset();
    m_timer.Start();
}

void Robot::AutonomousPeriodic() {
 
if (m_timer.Get() < 2_s) {
      // Drive forwards half speed
      m_leftLeadMotor.Set(0.5);
        m_leftFollowMotor.Set(0.5);
        m_rightLeadMotor.Set(0.5);
        m_rightFollowMotor.Set(0.5);
    } else {
      // Stop robot
       m_leftLeadMotor.Set(0);
        m_leftFollowMotor.Set(0);
        m_rightLeadMotor.Set(0);
        m_rightFollowMotor.Set(0);
    }        

}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {
  SmartDashboard::PutNumber("Encoder", m_leftEncoder.GetPosition());
    
  /******************************************************************************************************************************
  LIFT
  ******************************************************************************************************************************/

  if (CoPilot->GetPOV() == 90){
    sol_Climber.Set(frc::DoubleSolenoid::Value::kForward);
  }
  else if (CoPilot->GetPOV() == 270){
    sol_Climber.Set(frc::DoubleSolenoid::Value::kReverse);
  }

  if (CoPilot->GetPOV() == 0){
    m_leftLiftMotor.Set(0.5);
    m_rightLiftMotor.Set(0.5);
  }
  else if (CoPilot->GetPOV() == 180){
    m_leftLiftMotor.Set(-0.5);
    m_rightLiftMotor.Set(-0.5);
  }
  else {
    m_leftLiftMotor.Disable();
    m_rightLiftMotor.Disable();
  }

  
  /******************************************************************************************************************************
  INTAKE
  ******************************************************************************************************************************/

  if (CoPilot->GetRightTriggerAxis() >= SmartDashboard::GetNumber("Min Intake Percent", 0.5)){
    m_intakeFrontMotor.Set(CoPilot->GetAxisCount());
    m_intakeBackMotor.Set(CoPilot->GetAxisCount());
  }
  else if (CoPilot->GetLeftTriggerAxis()){
    m_intakeBackMotor.Set(-(CoPilot->GetAxisCount()));
    m_intakeFrontMotor.Set(-(CoPilot->GetAxisCount()));
  }
  else {
    m_intakeBackMotor.Set(0);
    m_intakeFrontMotor.Set(0);
  }

  if (CoPilot->GetAButtonPressed()){
    sol_Intake.Set(frc::DoubleSolenoid::Value::kForward);
  }
  
  else if (CoPilot->GetBButtonPressed()){
    sol_Intake.Set(frc::DoubleSolenoid::Value::kReverse);
  }

  /******************************************************************************************************************************
  SHOOTER
  ******************************************************************************************************************************/
  double shooterRPM = m_shooterMotorL.GetSelectedSensorVelocity() / 2048/*Units per rotation*/ * 10/*100ms to 1000ms/1s*/ * 60/*1s to 60s/1m*/ * shooterGearRatio;
  if (CoPilot->GetAButtonPressed()){
    shooterTargetRPM = SmartDashboard::GetNumber("shooter Far RPM", 5742 * shooterGearRatio * 0.8); 
    m_shooterPID.SetSetpoint(shooterTargetRPM);
  }
  else if (CoPilot->GetBButtonPressed()){
    shooterTargetRPM = SmartDashboard::GetNumber("shooter Tarmac RPM", 5742 * shooterGearRatio * 0.6);
    m_shooterPID.SetSetpoint(shooterTargetRPM);
  }
  //else if (cont_Partner->GetCircleButtonPressed()){
  //  shooterTargetRPM = SmartDashboard::GetNumber("shooter Fender RPM", 5742 * shooterGearRatio * 0.4);
  //  m_shooterPID.SetSetpoint(shooterTargetRPM);
  //}
  //else if (cont_Partner->GetCrossButtonPressed()){
  //  shooterTargetRPM = 0;
  //  m_shooterPID.SetSetpoint(shooterTargetRPM);
  //}
  SmartDashboard::PutNumber("shooter RPM", shooterRPM);
  SmartDashboard::PutNumber("shooter Target RPM", shooterTargetRPM);
  double output = std::clamp(m_shooterPID.Calculate(shooterRPM), shooterMinRPM, shooterMaxRPM);
  SmartDashboard::PutNumber("shooter Output", output);
  m_shooterMotorL.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, output);
  m_shooterMotorR.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, output); 
  
  /******************************************************************************************************************************
  DRIVE
  ******************************************************************************************************************************/
  float right = Pilot->GetRightY();
  float left = Pilot->GetLeftY();;

        m_leftLeadMotor.Set(left);
        m_leftFollowMotor.Set(left);
        m_rightLeadMotor.Set(right);
        m_rightFollowMotor.Set(right);
 
/* 
 if (Pilot->GetLeftY()){
    m_leftLeadMotor.Set(Pilot->GetAxisCount());
    m_leftFollowMotor.Set(Pilot->GetAxisCount());
  }
 else {
    m_leftLeadMotor.Set(0);
    m_leftFollowMotor.Set(0);
  }
if (Pilot->GetRightY()){
    m_rightLeadMotor.Set(Pilot->GetAxisCount());
    m_rightFollowMotor.Set(Pilot->GetAxisCount());
  }
 else {
    m_rightLeadMotor.Set(0);
    m_rightFollowMotor.Set(0);
  }  
  if (Pilot->GetRightBumperPressed()){
    sol_Shift.Set(frc::DoubleSolenoid::Value::kForward);
  }
  
  else if (Pilot->GetLeftBumperPressed()){
    sol_Shift.Set(frc::DoubleSolenoid::Value::kReverse);
  }
*/
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