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
Orchestra _orchestra;
string song = "short_imp.chrp";

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

rev::SparkMaxRelativeEncoder m_leftDriveEncoder = m_leftLeadMotor.GetEncoder();
rev::SparkMaxRelativeEncoder m_rightDriveEncoder = m_rightLeadMotor.GetEncoder();

frc::DifferentialDrive m_drive{m_leftLeadMotor, m_rightLeadMotor};
string currentDriveMode = "curve";
string altDriveMode = "tank";
frc2::PIDController driveControl{drivekP, drivekI, drivekD};
frc::SlewRateLimiter<units::scalar> m_speedLimiter{3 / 1_s};

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
frc::BangBangController m_shooterBangBang;
frc::SlewRateLimiter<units::revolutions_per_minute> m_shooterSlewLimiter{1500_rpm / 0.5_s};

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
static const int m_intakeFrontID = 21, m_intakeBackID = 20;
rev::CANSparkMax m_intakeFrontMotor{m_intakeFrontID, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax m_intakeBackMotor{m_intakeBackID, rev::CANSparkMax::MotorType::kBrushless};
rev::SparkMaxRelativeEncoder m_intakeFrontEncoder = m_intakeFrontMotor.GetEncoder();
rev::SparkMaxRelativeEncoder m_intakeBackEncoder = m_intakeBackMotor.GetEncoder();
DoubleSolenoid sol_Intake(1, frc::PneumaticsModuleType::REVPH, 4, 5);

DoubleSolenoid sol_limelight(1, frc::PneumaticsModuleType::REVPH, 6, 7);

//Lift and Climb
static const int m_leftLiftID = 31, m_rightLiftID = 30;
rev::CANSparkMax m_leftLiftMotor{m_leftLiftID, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax m_rightLiftMotor{m_rightLiftID, rev::CANSparkMax::MotorType::kBrushless};
rev::SparkMaxRelativeEncoder m_leftLiftEncoder = m_leftLiftMotor.GetEncoder();
rev::SparkMaxRelativeEncoder m_rightLiftEncoder = m_rightLiftMotor.GetEncoder();
DoubleSolenoid sol_Climber(1, frc::PneumaticsModuleType::REVPH, 2, 3);
rev::SparkMaxPIDController m_leftLiftPIDController = m_leftLiftMotor.GetPIDController();
rev::SparkMaxPIDController m_rightLiftPIDController = m_rightLiftMotor.GetPIDController();
double kLiftP = 0.1, kLiftI = 1e-4, kLiftD = 1, kLiftIz = 0, kLiftFF = 0, kLiftMaxOutput = 0.2, kLiftMinOutput = -0.2;
// units::volts kS = 3_V;
// units::volts kG = 6_V;


// ElevatorFeedforward<units::inches> liftFeedforward(kS, kG, kV, kA);


//Controllers
XboxController  *Pilot = new XboxController(0);
XboxController *CoPilot= new XboxController(1);
double joy_lStick_Y_deadband = 0.05, joy_rStick_Y_deadband = 0.05, joy_rStick_X_deadband = 0.05;

frc::Timer m_timer;

double getLeftEncoderDist(){
  return m_leftDriveEncoder.GetPosition() / 42/*ticks per rev*/ * driveGearRatio * (2 * M_PI * 0.0762)/*dist per rev in meters*/;
}

double getRightEncoderDist(){
  return m_rightDriveEncoder.GetPosition() / 42 * driveGearRatio * (2 * M_PI * 0.0762);
}
/*
void move(double dist){
  double rotations = (dist/(6*M_PI))*8.68;
  //DifferentialDrive::
}
*/

void Robot::MoveClimber(double rotations){

  // double currentLeftLiftPostion = m_leftLiftEncoder.GetPosition();
  // double currentRightLiftPostion = m_rightLiftEncoder.GetPosition();
  // m_leftLiftPIDController.SetReference(rotations, rev::ControlType::kPosition);
  // m_rightLiftPIDController.SetReference(rotations, rev::ControlType::kPosition);

}
void Robot::AutoShootAtTargetPRM(double rpm){
  m_intakeBackEncoder.SetPosition(0);
  if (m_shooterMotorL.GetSelectedSensorVelocity() < rpm ){ // Should this be a while loop?
    m_shooterMotorL.Set(ControlMode::Velocity, rpm);
    m_shooterMotorR.Set(ControlMode::Velocity, rpm);
  }
  else if(m_shooterMotorL.GetSelectedSensorVelocity() == rpm){
    double currentBackEncoderPostion = m_intakeBackEncoder.GetPosition();
    if (currentBackEncoderPostion < 2000 /* Revolutions to drive the intake for?? */){
      m_intakeBackMotor.Set(0.5); //Might need to be negative if this is the wrong direction (Probably should invert the controller so + = in)
    }
  }
}

void Robot::DriveRobot(double distance){
  // I'm not sure whether this is worth setting up until PID stuff is done. Previous Code had PID controller for distance

}

void Robot::DriveRobotForward(units::second_t time){
  //Probably should have gear shift check in here
  m_drive.TankDrive(0.5, 0.5);
  Wait(time);
  m_drive.TankDrive(0, 0);
}

void Robot::RotateRobot(units::degrees degrees){
  //Not sure how to handle this until gyro is working
}
void Robot::ExtendClimber(){
  sol_Climber.Set(DoubleSolenoid::kForward);
} 
void Robot::RetractClimber(){
  sol_Climber.Set(DoubleSolenoid::kReverse);
}
void Robot::ExtendIntake(){
  sol_Intake.Set(DoubleSolenoid::kReverse);  
}
void Robot::RetractIntake(){
  sol_Intake.Set(DoubleSolenoid::kForward);
}
void Robot::HighGear(){
  sol_Shift.Set(DoubleSolenoid::kReverse);
}
void Robot::LowGear(){
  sol_Shift.Set(DoubleSolenoid::kForward);
}

void Robot::RunIntake(units::second_t time){
  m_intakeFrontMotor.Set(0.9);
  m_intakeBackMotor.Set(0.9);
  Wait(time);
  m_intakeFrontMotor.Set(0);
  m_intakeBackMotor.Set(0);
}

void Robot::RunIntake(units::second_t time, bool invert){
  m_intakeFrontMotor.Set(-0.9);
  m_intakeBackMotor.Set(-0.9);
  Wait(time);
  m_intakeFrontMotor.Set(0);
  m_intakeBackMotor.Set(0);
}

void Robot::RunIntake(bool invert){
  m_intakeFrontMotor.Set(-0.9);
  m_intakeBackMotor.Set(-0.9);
}

void Robot::RunIntake(){
  m_intakeFrontMotor.Set(0.9);
  m_intakeBackMotor.Set(0.9);
}

void Robot::StopIntake(){
  m_intakeFrontMotor.Set(0);
  m_intakeBackMotor.Set(0);
}

void Robot::RunShooter(units::second_t time){
  m_shooterMotorL.Set(ControlMode::PercentOutput, 0.5);
  m_shooterMotorR.Set(ControlMode::PercentOutput, 0.5);
  Wait(time);
  m_shooterMotorL.Set(0);
  m_shooterMotorR.Set(0);
}

void Robot::RunShooter(units::second_t time, double percentOutput){
  m_shooterMotorL.Set(ControlMode::PercentOutput, percentOutput);
  m_shooterMotorR.Set(ControlMode::PercentOutput, percentOutput);
  Wait(time);
  m_shooterMotorL.Set(0);
  m_shooterMotorR.Set(0);
}

void Robot::RunShooter(double percentOutput){
  m_shooterMotorL.Set(ControlMode::PercentOutput, percentOutput);
  m_shooterMotorR.Set(ControlMode::PercentOutput, percentOutput);
}

void Robot::OnlyDriveAuto(){
  LowGear();
  ExtendIntake();
 
 RunIntake(true);
 Wait(0.2_s); // .5s
 StopIntake(); 

 RunShooter(.6);
 Wait(1_s); //1.5s
 RunIntake();
 Wait(3.5_s); //5s
 StopShooter();

 m_drive.TankDrive(-0.5,-0.5); //drives forwards
 Wait(3_s);
 m_drive.TankDrive(0,0);
  
StopIntake();
StopShooter();

Wait(3_s);

// while(m_timer.Get() < 15_s){
  // Wait(0.01_s);
// }

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
// while(m_timer.Get() < 10_s){
  // Wait(0.01_s);
  // }

}
void Robot::CSAAuto(){
  if (m_timer.Get() < .3_s) {
    LowGear();
    ExtendIntake();
    //  Wait(.3_s); // .3s 
  }
  else if (m_timer.Get() < .5_s) {
    RunIntake(true);
    // Wait(0.2_s); // .5s
  }
  else if (m_timer.Get() < 1.5_s) {
    StopIntake();

    RunShooter(.585);
    // Wait(1_s); //1.5s
  }
  else if (m_timer.Get() < 5_s) {
    RunIntake();
    //  Wait(3.5_s); //5s
  }
  else if (m_timer.Get() < 7.75_s) {
    RunShooter(.7);

    m_drive.TankDrive(-0.5,-0.5); //drives forwards + intake
    // Wait(2.75_s); //7s
  }
  else if (m_timer.Get() < 8.45_s) {
    m_drive.TankDrive(0.5, 0.5);
    // Wait(0.7s);
  }
  else if (m_timer.Get() < 15_s) {
    m_drive.TankDrive(0,0);
    StopIntake();
    StopShooter();

    // Wait(7_s);
    // while(m_timer.Get() < 10_s){
      // Wait(0.01_s);
      // }
  }
  else {

  }
}


void Robot::StopShooter(){
  m_shooterMotorL.Set(0);
  m_shooterMotorR.Set(0);
}

void Robot::Rainbow() {
    // For every pixel
    for (int i = 0; i < kLength; i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      const auto pixelHue = (firstPixelHue + (i * 180 / kLength)) % 180;
      // Set the value
      m_ledBuffer[i].SetHSV(pixelHue, 255, 128);
    } 
    // Increase by to make the rainbow "move"
    firstPixelHue += 3;
    // Check bounds
    firstPixelHue %= 180;
  }



void Robot::RobotInit() {

 // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update data
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
  m_chooser.SetDefaultOption("Standard Auto", "standard");
  m_chooser.AddOption("Only Drive","drive");
  SmartDashboard::PutData(&m_chooser);

  // m_chooser2.SetDefaultOption("Stevie Drive", "stevie");
  // m_chooser2.AddOption("Gavin Drive", "gavin");
  // SmartDashboard::PutData(&m_chooser2);

}


void Robot::RobotPeriodic() {
  //position = odometry->Update(getGyroAngle(), units::meter_t(getLeftEncoderDist()), units::meter_t(getRightEncoderDist()));
  //autoAimPID.SetSetpoint(0);
}
/******************************************************************************************************************************
                                             ###    ##     ## ########  #######  
                                            ## ##   ##     ##    ##    ##     ## 
                                           ##   ##  ##     ##    ##    ##     ## 
                                          ##     ## ##     ##    ##    ##     ## 
                                          ######### ##     ##    ##    ##     ## 
                                          ##     ## ##     ##    ##    ##     ## 
                                          ##     ##  #######     ##     #######
                                                      AUTO                                                                                             
******************************************************************************************************************************/
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

void Robot::AutonomousPeriodic() {
  string selected_auto = m_chooser.GetSelected();
  // if (selected_auto == "standard"){
    DefaultAuto();
  // }
  // else if (selected_auto == "drive"){
  //  OnlyDriveAuto();
//  }

}

void Robot::TeleopInit() {
  m_drive.SetSafetyEnabled(true);
}

//THESE VALUES MAY NEED TO GO IN TELEOPINIT
double LLSteerAdjust = 0;
double LLDriveAdjust = 0;
double LLShooterSpeed = 0;
//THESE VALUES MAY NEED TO GO IN TELEOPINIT


void Robot::TeleopPeriodic() {
/********************************************************************************************************************************
                                  ##                ##
                                  ##                ##
                                  ##                ##
                                  ##                ##    
                                  ##                ##
                                  ##                ##
                                  ##########        ##########          
                                            LIMELIGHT
 *******************************************************************************************************************************/ 

  double tx = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx", 0.0);
  double ty = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("ty", 0.0);
  double ta = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("ta", 0.0);
  double tv = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tv", 0.0);
  double thor = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("thor", 0.0);  
  double tvert = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tvert", 0.0);
  //Turn & Driving Tracking
  if (thor > tvert){ //if target bounding box is a rectangle
     tx = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx", 0.0);
     ty = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("ty", 0.0);
     ta = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("ta", 0.0);
     thor = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("thor", 0.0);  
     tvert = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tvert", 0.0);
    
    if(tx < -8){ //target is too far left (need to turn right (positive))
      tx = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx", 0.0);
      ty = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("ty", 0.0);
      ta = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("ta", 0.0);
      thor = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("thor", 0.0);  
      tvert = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tvert", 0.0);
      
      LLSteerAdjust = tx*-.02; //target x value times -.02 (makes a positive turn value)
      m_drive.CurvatureDrive(LLDriveAdjust, LLSteerAdjust, true);
      
    } else if(tx > 8){ //target is too far right (need to turn left (negative))
      tx = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx", 0.0);
      ty = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("ty", 0.0);
      ta = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("ta", 0.0);
      thor = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("thor", 0.0);  
      tvert = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tvert", 0.0);
      
      LLSteerAdjust = tx*-.02; 
      m_drive.CurvatureDrive(LLDriveAdjust, LLSteerAdjust, true); 
      
    } else if(tx > -8 && tx < 8){ //target is alligned 
      tx = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx", 0.0);
      ty = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("ty", 0.0);
      ta = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("ta", 0.0);
      thor = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("thor", 0.0);  
      tvert = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tvert", 0.0);
      
      LLSteerAdjust = 0;
      m_drive.CurvatureDrive(LLDriveAdjust, LLSteerAdjust, true);
      
    } //end if tx loop
    
    if(ta > .2){ //target is too close
      tx = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx", 0.0);
      ty = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("ty", 0.0);
      ta = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("ta", 0.0);
      thor = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("thor", 0.0);  
      tvert = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tvert", 0.0);
      
      LLDriveAdjust = -.15;
      LLShooterSpeed = (ta*-.005) +.5;
      m_drive.CurvatureDrive(LLDriveAdjust, LLSteerAdjust, true); //go forwards
      //tell shooter to run here
      
    } else if (ta < .02){ //target is too far
      tx = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx", 0.0);
      ty = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("ty", 0.0);
      ta = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("ta", 0.0);
      thor = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("thor", 0.0);  
      tvert = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tvert", 0.0);
      
      LLDriveAdjust = .15;
      LLShooterSpeed = (ta*-.005) +.5;
      m_drive.CurvatureDrive(LLDriveAdjust, LLSteerAdjust, true); //go backwards
      //tell shooter to run here
      
    } else if (ta > .05 && ta <.2){ //target is in range
      tx = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx", 0.0);
      ty = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("ty", 0.0);
      ta = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("ta", 0.0);
      thor = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("thor", 0.0);  
      tvert = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tvert", 0.0);
      
      LLDriveAdjust = 0;
      LLShooterSpeed = (ta*-.005) +.5;
      m_drive.CurvatureDrive(LLDriveAdjust, LLSteerAdjust, true);
      //tell shooter to run here
    
    } //end if ta loop
    
    if(ta>.05 && ta<.2 && tx>-8 && tx<8) { // target is aligned and in range
      tx = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx", 0.0);
      ty = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("ty", 0.0);
      ta = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("ta", 0.0);
      thor = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("thor", 0.0);  
      tvert = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tvert", 0.0);
     
      LLSteerAdjust = 0;
      LLDriveAdjust = 0;
      LLShooterSpeed = (ta*-.005) + .5;
      m_drive.CurvatureDrive(LLDriveAdjust, LLSteerAdjust, true);
      //tell shooter to run here
      
    } //end if tx AND ta loop

    
//     if(){
//      
//     }
  
  }//end if rectangle loop  
    
  /******************************************************************************************************************************
                                                ##       #### ######## ######## 
                                                ##        ##  ##          ##    
                                                ##        ##  ##          ##    
                                                ##        ##  ######      ##    
                                                ##        ##  ##          ##    
                                                ##        ##  ##          ##    
                                                ######## #### ##          ## 
                                                          LIFT   
  ******************************************************************************************************************************/
  // All Smart Dashboard Values will attempt to be placed at the beginning of each Subsystem Section

  // SmartDashboard::PutNumber("Left Climber Position ", m_leftLiftEncoder.GetPosition());
  // SmartDashboard::PutNumber("Right Climber Position ", m_rightLiftEncoder.GetPosition());
 
  if (CoPilot->GetPOV() == 90){
    RetractClimber();
  }
  else if (CoPilot->GetPOV() == 270){
    ExtendClimber();
  }

// double leftLift = CoPilot->GetLeftY();

//     m_leftLiftMotor.Set(1*leftLift);
//     m_rightLiftMotor.Set(1*leftLift);

double leftLift = CoPilot->GetLeftY();
double rightLift = CoPilot->GetRightY();

m_leftLiftMotor.Set(rightLift);
m_rightLiftMotor.Set(-1*leftLift);






  if (CoPilot->GetStartButtonPressed()){
    // Partymodeyeahwoo();
    for (int i = 0; i < 15; i++) {
      for (int i = 0; i < kLength; i++) {
        m_ledBuffer[i].SetRGB(255,0,0);}
        m_led.SetData(m_ledBuffer);
        Wait(.2_s);
      for (int i = 0; i < kLength; i++) {
        m_ledBuffer[i].SetRGB(0,255,0);}
        m_led.SetData(m_ledBuffer);
        Wait(.2_s);
      for (int i = 0; i < kLength; i++) {
        m_ledBuffer[i].SetRGB(0,0,255);}
        m_led.SetData(m_ledBuffer);
        Wait(.2_s);
    }
  }

  /******************************************************************************************************************************
                                      #### ##    ## ########    ###    ##    ## ######## 
                                       ##  ###   ##    ##      ## ##   ##   ##  ##       
                                       ##  ####  ##    ##     ##   ##  ##  ##   ##       
                                       ##  ## ## ##    ##    ##     ## #####    ######   
                                       ##  ##  ####    ##    ######### ##  ##   ##       
                                       ##  ##   ###    ##    ##     ## ##   ##  ##       
                                      #### ##    ##    ##    ##     ## ##    ## ######## 
                                                          INTAKE
  ******************************************************************************************************************************/
  // SmartDashboard::PutNumber("Current Front Intake Velocity", m_intakeFrontEncoder.GetVelocity());
  // SmartDashboard::PutNumber("Current Back Intake Velocity", m_intakeBackEncoder.GetVelocity());

  // if (CoPilot->GetRightTriggerAxis() >= SmartDashboard::GetNumber("Min Intake Percent", 0.5)){
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
  if (CoPilot->GetXButton()){
    m_intakeFrontMotor.Set(0.75);
  }

  if (Pilot->GetLeftStickButtonPressed()){
    RetractIntake();
  }
  
  else if (Pilot->GetRightStickButtonPressed()){
    ExtendIntake();
  }
//
//
//
  if (CoPilot->GetLeftBumperPressed()){
    RetractIntake();
  }

  else if (CoPilot->GetRightBumperPressed()){
    ExtendIntake();
  }

  /******************************************************************************************************************************
                               ######  ##     ##  #######   #######  ######## ######## ########  
                              ##    ## ##     ## ##     ## ##     ##    ##    ##       ##     ## 
                              ##       ##     ## ##     ## ##     ##    ##    ##       ##     ## 
                               ######  ######### ##     ## ##     ##    ##    ######   ########  
                                    ## ##     ## ##     ## ##     ##    ##    ##       ##   ##   
                              ##    ## ##     ## ##     ## ##     ##    ##    ##       ##    ##  
                               ######  ##     ##  #######   #######     ##    ######## ##     ##  
                                                          SHOOTER
  ******************************************************************************************************************************/
  double shooterRPM = m_shooterMotorL.GetSelectedSensorVelocity() / 2048/*Units per rotation*/ * 10/*100ms to 1000ms/1s*/ * 60/*1s to 60s/1m*/ * shooterGearRatio;
  // double targetVelocity_Per100ms = 2000 * 2048 / 600; //Gives weird numbers so we aren't using this anymore (Maybe fixed?)
  double targetVelocity_Per100ms = 5100;
  // SmartDashboard::PutNumber("Shooter RPM", shooterRPM);
  // SmartDashboard::PutNumber("Shooter Target RPM", targetVelocity_Per100ms * 600 / 2048);
  double output = std::clamp(m_shooterPID.Calculate(shooterRPM), shooterMinRPM, shooterMaxRPM);
  // SmartDashboard::PutNumber("Shooter Output", output);
 

  if (CoPilot->GetBButton()){
    // shooterTargetRPM = SmartDashboard::GetNumber("shooter Far RPM", 5742 * shooterGearRatio * 0.8); 
    // m_shooterPID.SetSetpoint(shooterTargetRPM);
    for (int i = 0; i < kLength; i++) {
   m_ledBuffer[i].SetRGB(0,255,0);}
  m_led.SetData(m_ledBuffer);

    m_shooterMotorL.Set(ControlMode::Velocity, targetVelocity_Per100ms);
    m_shooterMotorR.Set(ControlMode::Velocity, targetVelocity_Per100ms);
  }
  else if (CoPilot->GetAButton()){
    // shooterTargetRPM = SmartDashboard::GetNumber("shooter Tarmac RPM", 5742 * shooterGearRatio * 0.6);
    // m_shooterPID.SetSetpoint(shooterTargetRPM);
    for (int i = 0; i < kLength; i++) {
   m_ledBuffer[i].SetRGB(0,0,255);}
  m_led.SetData(m_ledBuffer);
 
    m_shooterMotorL.Set(ControlMode::PercentOutput, 0.40);
    m_shooterMotorR.Set(ControlMode::PercentOutput, 0.40);
  }
  // else if (CoPilot->GetXButton()){
    // shooterTargetRPM = SmartDashboard::GetNumber("Shooter Tarmac RPM", 5742 * shooterGearRatio * 0.6);
    // m_shooterMotorL.Set(m_shooterSlewLimiter.Calculate(shooterRPM, shooterTargetRPM));
    // m_shooterMotorR.Set(m_shooterBangBang.Calculate(shooterRPM, shooterTargetRPM));
  //}
  else {
    m_shooterMotorL.Set(0);
    m_shooterMotorR.Set(0);
for (int i = 0; i < kLength; i++) {
   m_ledBuffer[i].SetRGB(255,0,0);}
  m_led.SetData(m_ledBuffer);
  }
  // m_shooterMotorL.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, output);
  // m_shooterMotorR.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, output); 
  
  /******************************************************************************************************************************
                                      ########  ########   #### ##     ## ######## 
                                      ##     ## ##     ##   ##  ##     ## ##       
                                      ##     ## ##     ##   ##  ##     ## ##       
                                      ##     ## ########    ##  ##     ## ######   
                                      ##     ## ##   ##     ##   ##   ##  ##       
                                      ##     ## ##    ##    ##    ## ##   ##       
                                      ########  ##     ##  ####    ###    ######## 
                                                        DRIVE
  ******************************************************************************************************************************/
  double turningRate;
  double rightJoystick = Pilot->GetRightY();
  double leftJoystick = Pilot->GetLeftY();
  SmartDashboard::PutNumber("Left Drive Velocity", -1 * m_leftDriveEncoder.GetVelocity());
  SmartDashboard::PutNumber("Right Drive Velocity", -1 * m_rightDriveEncoder.GetVelocity());
  SmartDashboard::PutString("Current Drive Mode", currentDriveMode);
  turningRate = (-1*(Pilot->GetRightTriggerAxis())) + Pilot->GetLeftTriggerAxis();

  double triggerslowturn = (-.9 * (Pilot->GetRightX()) + ((.2 * Pilot->GetLeftTriggerAxis()) - (.2 * Pilot->GetRightTriggerAxis())));

  if (currentDriveMode == "tank"){
    m_drive.TankDrive(leftJoystick, rightJoystick, true);
  }
  else if (currentDriveMode == "curve"){ 
      // m_drive.CurvatureDrive((Pilot->GetLeftTriggerAxis() - Pilot->GetRightTriggerAxis()), (turningRate*.1)+(-1 * Pilot->GetRightX()), true);
      m_drive.CurvatureDrive(( .9 * Pilot->GetLeftY()), triggerslowturn, true);
  }

  // if (Pilot->GetXButtonPressed()){
  //   currentDriveMode.swap(altDriveMode);
  // }

  if (Pilot->GetYButtonPressed()){
    // leftJoystick = -1*leftJoystick;
  }

  if (Pilot->GetAButtonPressed()){
    LowGear();
    //Low Gear
  }
  
  else if (Pilot->GetBButtonPressed()){
    HighGear();
    //High Gear
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
