/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

#include <iostream>
#include <cmath>

#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() 
{
  drvstk = new frc::Joystick(0);
  shtstk = new frc::Joystick(1);
  bttnswtcher =new frc::Joystick(2);
  bttnjoy = new frc::Joystick(3);
  ldrv.SetInverted(true);
  rdrv.SetInverted(true);
  //lenc.SetPositionConversionFactor(8);
  //renc.SetPositionConversionFactor(8);
  ldrvflw.Follow(ldrv, false);
  rdrvflw.Follow(rdrv, false);
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
  gyro.Calibrate();
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() 
{
  Update_Inputs();
}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() 
{
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  //std::cout << "Auto selected: " << m_autoSelected << std::endl;
  /*
  if (m_autoSelected == kAutoNameCustom)
  {
    // Custom Auto goes here
  } 
  else 
  {
    // Default Auto goes here
  }
  */
  Auto1.ControlVar.shtspeed=0;
  Auto1.ControlVar.rspeed=0;
  Auto1.ControlVar.lspeed=0;

  if(autostpos)autonum=1;
  ldrv.GetEncoder().SetPosition(0);
  rdrv.GetEncoder().SetPosition(0);
}

void Robot::AutonomousPeriodic() 
{
  if (m_autoSelected == kAutoNameCustom) 
  {
    // Custom Auto goes here
  } 
  else 
  {
    // Default Auto goes here
  }
  blt1.Set(ControlMode::PercentOutput, -Auto1.ControlVar.shtspeed);
  blt2.Set(ControlMode::PercentOutput, Auto1.ControlVar.shtspeed);
  rvrblt.Set(ControlMode::PercentOutput, Auto1.ControlVar.shtspeed);
  Drivetrain.TankDrive(Auto1.ControlVar.lspeed,Auto1.ControlVar.rspeed,false);  
  
  if(autodostuff)
  {
    switch (autonum)
    {
    case 0: //close start
      {
        Auto1.RunAutoPhase(4);
        /*
        Auto1.RunAutoPhase(0);
        if(Auto1.GetPhaseDone())
        {
          if(autotrvl&&autotrvlmthd)
            autonum=2;
          else if(autotrvl&&!autotrvlmthd)
            autonum=3;
          else
            autonum=10;
          Auto1.AdvancePhase();
        }
        */
        break;
      }
    
    case 1: //far start
    {
      Auto1.RunAutoPhase(1);
      if(Auto1.GetPhaseDone())
      {
        if(autotrvl&&autotrvlmthd)
          autonum=2;
        else if(autotrvl&&!autotrvlmthd)
          autonum=3;
        else
          autonum=10;
        Auto1.AdvancePhase();
      }
      break;
    }
    case 2: //shield
    {
      Auto1.RunAutoPhase(2);
      if(Auto1.GetPhaseDone())
        {
          autonum=10;
          Auto1.AdvancePhase();
        }
      break;
    }
    case 3: //trench
    {
      Auto1.RunAutoPhase(3);
      if(Auto1.GetPhaseDone())
        {
          autonum=10;
          Auto1.AdvancePhase();
        }
      break;
    }
    case 10:
    {
      wpi::errs() << "Autonomous Sequence Complete";
      autonum++;
    }
    default:
      break;
    }
  } 

  

  
}

void Robot::TeleopInit() 
{

}

void Robot::TeleopPeriodic()
{
  Drivetrain.ArcadeDrive(-drvstk->GetY()*thrttlctrl,-drvstk->GetX()*thrttlctrl,true);

  if(cmprsractv&&drvstk->GetRawButton(11))
  {
    cmprsr.Stop();
    cmprsractv=false;
  }
  else if(!cmprsractv&&drvstk->GetRawButton(11))
  {
    cmprsr.Start();
    cmprsractv=true;
  }
  if(shtstk->GetRawButton(7)||bttnswtcher->GetRawButton(10))
    Color_Match(clrtgt, clrvw) ? spnr.Set(ControlMode::PercentOutput, 0) : spnr.Set(ControlMode::PercentOutput, .5);
  else if(shtstk->GetRawButton(6)||bttnswtcher->GetRawButton(11))
  {
    if(!spnrencReset)
      {
        spnrenc.Reset();
        spnrencReset=true;
      }
    spnrMotion.ConvertInchTick(4,2048);
    if(spnrenc.Get()<=12144)
    spnr.Set(ControlMode::PercentOutput, 1);
    else if(spnrenc.Get()<=12650)
    spnr.Set(ControlMode::PercentOutput, .75);
    else spnr.Set(ControlMode::PercentOutput, 0);
  }
  else
    {
      spnr.Set(ControlMode::PercentOutput, 0);
      spnrencReset=false;
    }

  if(shtstk->GetRawButton(5)||bttnswtcher->GetRawButton(12)) //+ direction
  {
    blt1.Set(ControlMode::PercentOutput, -shtctrl);
    blt2.Set(ControlMode::PercentOutput, shtctrl);
    rvrblt.Set(ControlMode::PercentOutput, shtctrl);
  }
  else
  {
      blt1.Set(ControlMode::PercentOutput, 0);
      blt2.Set(ControlMode::PercentOutput, 0);
      rvrblt.Set(ControlMode::PercentOutput, 0);
  }

  if(shtstk->GetRawButton(2)||bttnswtcher->GetY()>=0.5)
  {
    hkdpl.Set(frc::DoubleSolenoid::kForward);
  }
  else if(shtstk->GetRawButton(3)||bttnswtcher->GetY()<= -0.5)
  {
    hkdpl.Set(frc::DoubleSolenoid::kReverse);
  }
  else
  {
    hkdpl.Set(frc::DoubleSolenoid::kOff);
  }
  
  if(shtstk->GetRawButton(6))
  {
    mstdpl.Set(ControlMode::PercentOutput, thrttlctrl*shtstk->GetY());
    wnchl.Set(std::abs(-0.5));
    wnchr.Set(std::abs(-0.5)); 
  }
  else if(bttnjoy->GetY()<=-0.5)
  {
    mstdpl.Set(ControlMode::PercentOutput, thrttlctrl*bttnjoy->GetY());
    wnchl.Set(std::abs(-0.3));
    wnchr.Set(std::abs(-0.3)); 
  }
  else
  {
    mstdpl.Set(ControlMode::PercentOutput, thrttlctrl*bttnjoy->GetY());
  }
  
  if(shtstk->GetRawButton(4))
  {
    mstdpl.Set(ControlMode::PercentOutput, thrttlctrl*shtstk->GetY());
  }
  else if(bttnjoy->GetY()>=0.5)
  {
    mstdpl.Set(ControlMode::PercentOutput, thrttlctrl*bttnjoy->GetY());
  }
  else
  {
    mstdpl.Set(ControlMode::PercentOutput, 0);
  }
  
  if(shtstk->GetRawButton(1))
  {
    wnchl.Set(-shtctrl*std::abs(shtstk->GetY()));
    wnchr.Set(-shtctrl*std::abs(shtstk->GetY()));
  }
  else if(shtstk->GetRawButton(10))
  {
    wnchl.Set(-shtctrl*shtstk->GetY());
    wnchr.Set(-shtctrl*shtstk->GetY());
  }
  else if(bttnjoy->GetRawButton(12))
  {
    wnchl.Set(-0.45);
    wnchr.Set(-0.45);
  }
  else
  {
    wnchl.Set(0);
    wnchr.Set(0);
  }
  
  if (shtstk->GetRawButton(11))
  {
    clrdpl.Set(true);
  }
  else
  {
    clrdpl.Set(false);
  }

  if(drvstk->GetRawButton(8))
  {
    ldrv.GetEncoder().SetPosition(0);
    rdrv.GetEncoder().SetPosition(0);
  }
  
}

void Robot::TestPeriodic() 
{

}

bool Robot::Color_Match(rev::ColorSensorV3::RawColor clrtgt, rev::ColorSensorV3::RawColor clrvw)
{
  const double m_tol=10;
  return (abs(clrvw.red-clrtgt.red)<m_tol && abs(clrvw.green-clrtgt.green)<m_tol && abs(clrvw.blue-clrtgt.blue)<m_tol);
}

void Robot::Update_Inputs()
{

  Auto1.SensorVar.ldrvPos=lenc.GetPosition();
  Auto1.SensorVar.rdrvPos=-renc.GetPosition();
  Auto1.SensorVar.gangle=gyro.GetAngle();
  //autostpos=//posswtch.Get();
  //autotrvlmthd=//mthdswtch.Get();
  //autotrvl=//trvlswtch.Get();
  //autodostuff=//mstrswtch.Get();
  thrttlctrl=(drvstk->GetZ()+1)/2;
  shtctrl=(shtstk->GetZ()+1)/2;
  

  
  frc::SmartDashboard::PutBoolean("Auto Phase Complete",Auto1.GetPhaseDone());
  frc::SmartDashboard::PutBoolean("Auto Master Active", autodostuff);
  frc::SmartDashboard::PutBoolean("AutoPos", autostpos);
  frc::SmartDashboard::PutBoolean("Auto Trvl Method", autotrvlmthd);
  frc::SmartDashboard::PutBoolean("Auto Trvl", autotrvl);
  
  frc::SmartDashboard::PutNumber("Throttle Control",thrttlctrl);
  frc::SmartDashboard::PutNumber("Shooter Throttle Control", shtctrl);
  frc::SmartDashboard::PutNumber("Left Encoder:",Auto1.SensorVar.ldrvPos);
  frc::SmartDashboard::PutNumber("Right Encoder:",Auto1.SensorVar.rdrvPos);
  frc::SmartDashboard::PutNumber("Sensor Color: Red", clrprc.red);
  frc::SmartDashboard::PutNumber("Sensor Color: Green", clrprc.green);
  frc::SmartDashboard::PutNumber("Sensor Color: Blue", clrprc.blue);
  frc::SmartDashboard::PutNumber("Spinner Encoder",spnrenc.Get());
  frc::SmartDashboard::PutNumber("Mast Encoder",mstenc.Get());
  
  frc::SmartDashboard::PutNumber("autonum", autonum);
  //frc::SmartDashboard::


  gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
  if(gameData.length() > 0)
  {
    switch (gameData[0])
    {
    case 'B' : //Cyan
      clrtgt.red=255;
      break;
    case 'G' :
      clrtgt.red=255;
      clrtgt.green=255;
      break;
    case 'R' :
      clrtgt.blue=255;
      clrtgt.green=255;
      break;
    case 'Y' :
      clrtgt.red=255;
      clrtgt.green=255;
      break;
    default :
      //This is corrupt data
      break;
      }
  }  
  else 
  {
  //Code for no data received yet
  }
  clrvw =clrsnr.GetRawColor();
  clrvw.green=clrvw.green*3/4;
  double clrmax=clrvw.red+clrvw.green+clrvw.blue;
  clrprc.red=clrvw.red/clrmax;
  clrprc.green=clrvw.green/clrmax;
  clrprc.blue=clrvw.blue/clrmax;
}
#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
