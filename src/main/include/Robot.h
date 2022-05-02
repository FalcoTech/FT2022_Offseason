// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include "frc/AddressableLED.h"

class Robot : public frc::TimedRobot {
 public:

 static constexpr int kLength = 180;



  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;
  //void SimulationInit() override;
  //void SimulationPeriodic() override;
  void MoveClimber(double rotations);
  void AutoShootAtTargetPRM(double rpm);
  void DriveRobot(double distance);
  void DriveRobotForward(units::second_t time);
  void RotateRobot(units::degrees degrees);
  void ExtendClimber();
  void RetractClimber();
  void ExtendIntake();
  void RetractIntake();
  void HighGear();
  void LowGear();
  void RunIntake(units::second_t time);
  void RunIntake(units::second_t time, bool invert);
  void RunIntake();
  void RunIntake(bool invert);
  void StopIntake();
  void RunShooter(units::second_t time);
  void RunShooter(units::second_t time, double percentOutput);
  void RunShooter(double percentOutput);
  void StopShooter();
  void RunShooterAtRPM(units::second_t time, double rpm);
  void RunShooterAtRPM(double rpm);
  void DefaultAuto();
  void OnlyDriveAuto();

  void CSAAuto();

  //LEDS:

    // PWM port 9
  // Must be a PWM header, not MXP or DIO
  frc::AddressableLED m_led{9};
  std::array<frc::AddressableLED::LEDData, kLength>
      m_ledBuffer;  // Reuse the buffer
  // Store what the last hue of the first pixel is
  int firstPixelHue = 0;
   void Rainbow();
  //  void Partymodeyeahwoo();
 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;

  frc::SendableChooser<std::string> m_chooser2;
  const std::string kDriveNameDefault = "Stevie";
  const std::string kDriveNameCustom = "Gavin";
  std::string m_drivestyleSelected;
};
