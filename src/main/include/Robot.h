/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <string>

//#include "frc/WPILib.h"
#include <frc/drive/DifferentialDrive.h>
#include "ctre/Phoenix.h"
#include <rev/CANSparkMax.h>
#include <frc/SpeedController.h>
#include <frc/SpeedControllerGroup.h>
#include <frc/Spark.h>
#include <frc/Joystick.h>
#include <frc/Timer.h>
#include <frc/encoder.h>
#include <rev/ColorSensorV3.h>
#include <frc/Solenoid.h>
#include <frc/DoubleSolenoid.h>
#include <frc/Compressor.h>
#include <frc/DigitalInput.h>
#include <adi/ADIS16470_IMU.h>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/DriverStation.h>
#include "Motion.h"
#include "ModularAuto.h"


class Robot : public frc::TimedRobot 
{
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;
   
 private:
  
  std::string gameData;
  rev::ColorSensorV3::RawColor clrtgt =rev::ColorSensorV3::RawColor(0,0,0,0);
  rev::ColorSensorV3::RawColor clrvw =rev::ColorSensorV3::RawColor(0,0,0,0);
  frc::Color clrprc =frc::Color(0.0,0.0,0.0);
  const double INTTK= 2.22817;
  double ldrvPos=0;
  double rdrvPos=0;
  double lspeed=0;
  double rspeed=0;
  double thrttlctrl=1;
  double shtctrl=1;
  struct {double DrvX; double DrvY;}JoystickVals;
  
  int autonum=0;
  bool autostpos=false; //true==far pos | false==close pos
  bool autotrvlmthd=false; //false==trench | true==shieldgen
  bool autotrvl=false;
  bool autodostuff=true;
  bool cmprsractv=true;
  bool spnrencReset=false;
  bool isRecording=false;

  //Motion lMotion = Motion(0.02);
  //Motion rMotion = Motion(0.02);

  ModularAuto Auto1 = ModularAuto();

  
  //joysticks
  frc::Joystick* drvstk;
  frc::Joystick* shtstk;
  frc::Joystick* bttnjoy;
  frc::Joystick* bttnswtcher;
  //motors
  rev::CANSparkMax ldrv{1,rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax ldrvflw{2,rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax rdrv{3,rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax rdrvflw{4,rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax wnchl{5,rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax wnchr{6,rev::CANSparkMax::MotorType::kBrushless};

  TalonSRX spnr{11};
  TalonSRX mstdpl{7};
  TalonSRX rvrblt{8};
  TalonSRX blt1{9};
  TalonSRX blt2{10};

  
  //Compressor
  frc::Compressor cmprsr;
  //Solenoids
  frc::Solenoid clrdpl{0};
  //frc::Solenoid clrrtct{1};
  frc::DoubleSolenoid hkdpl{3,4};
  

  //Sensors
  rev::ColorSensorV3 clrsnr {frc::I2C::Port()};
  frc::ADIS16470_IMU gyro {};
  
  frc::DifferentialDrive Drivetrain {ldrv,rdrv};

  //Switches
  frc::DigitalInput posswtch{0};
  frc::DigitalInput mthdswtch{1};
  frc::DigitalInput trvlswtch{2};
  frc::DigitalInput mstrswtch{3};

  //ModularAuto Auto1 = ModularAuto();
  //Motion Control
  Motion spnrMotion= Motion(0.05);

  //Encoders
  rev::CANEncoder lenc = ldrv.GetEncoder(rev::CANEncoder::EncoderType::kHallSensor, 42);
  rev::CANEncoder renc = rdrv.GetEncoder(rev::CANEncoder::EncoderType::kHallSensor, 42);
  frc::Encoder spnrenc{4, 5, false, frc::Encoder::k4X};
  frc::Encoder mstenc{8, 9, false, frc::Encoder::k4X};

  bool Color_Match(rev::ColorSensorV3::RawColor clrtgt,rev::ColorSensorV3::RawColor clrvw);
  void Spin_Count();
  void Update_Inputs();
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
};
