// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <iostream>
#include <string>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <ctre/Phoenix.h>
#include <frc/PneumaticsControlModule.h>
#include <frc/Solenoid.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableInstance.h"
#include <cmath>


class Robot : public frc::TimedRobot {
  
  //drive train motors
  WPI_TalonFX _rghtFront{1};
	WPI_TalonFX _rghtFollower{3};
	WPI_TalonFX _leftFront{2};
	WPI_TalonFX _leftFollower{0};

  frc::PWMSparkMax _colector{1};
  frc::PWMSparkMax _belt{0};
  
  //drive train instance
	frc::DifferentialDrive _diffDrive{_leftFront, _rghtFront};

  //controllers
  frc::XboxController m_driverController{0};

  //network table instance
  std::shared_ptr<nt::NetworkTable> NetworkTable;  

  //penomatic systeam
  frc::Solenoid intake_solenoid{frc::PneumaticsModuleType::CTREPCM, 0};

  double CalcDistance(){
    // distance = (VisionTargetHeghit - CameraLensHeghit) / tan(CameraAngle+TragetLensAngle) --- the tan in radions make the converion  
    double yAngle = NetworkTable->GetNumber("ty",0);
    return (  (84-24) / tan(  (33+yAngle)*M_PI/180  )  );
  }

  void deployIntake(bool deploy){
    intake_solenoid.Set(deploy);
  }


  void RobotInit() override {
    
    auto inst = nt::NetworkTableInstance::GetDefault();
    NetworkTable = inst.GetTable("limelight");
    
    /* factory default values */
		_rghtFront.ConfigFactoryDefault();
		_rghtFollower.ConfigFactoryDefault();
		_leftFront.ConfigFactoryDefault();
		_leftFollower.ConfigFactoryDefault();

		/* set up followers */
		_rghtFollower.Follow(_rghtFront);
		_leftFollower.Follow(_leftFront);

		/* [3] flip values so robot moves forward when stick-forward/LEDs-green */
		_rghtFront.SetInverted(TalonFXInvertType::Clockwise);
		_rghtFollower.SetInverted(TalonFXInvertType::FollowMaster);
		_leftFront.SetInverted(TalonFXInvertType::CounterClockwise);
		_leftFollower.SetInverted(TalonFXInvertType::FollowMaster);



  
  }

  void TeleopPeriodic() override {    

    //this line of code is for the chassis moving for the Xbox controller stick
    _diffDrive.ArcadeDrive(-m_driverController.GetRawAxis(1)*1,
                             m_driverController.GetRawAxis(0)*1,
                             false);

    bool buut = m_driverController.GetRawButton(1);
    bool buut1 = m_driverController.GetRawButton(2);
    if(buut){
      //Testing shoting mode 
      NetworkTable->PutNumber("pipeline", 0);
    }
    else if(buut1){
      //Testing Driving mode
      deployIntake(1);
      NetworkTable->PutNumber("pipeline", 1);
    }

    _colector.Set(1);
    _belt.Set(1);
      
    frc::SmartDashboard::PutNumber("Distance", CalcDistance());
    
  }
};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
