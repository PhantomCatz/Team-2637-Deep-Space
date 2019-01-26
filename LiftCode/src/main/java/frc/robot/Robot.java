/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class Robot extends TimedRobot {
  public XboxController xbox;
  public CANSparkMax left;
  public CANSparkMax right;

  public DifferentialDrive liftDrive;


  @Override
  public void robotInit() {
    xbox = new XboxController(0);
    left = new CANSparkMax(10, MotorType.kBrushless);
    right = new CANSparkMax(11, MotorType.kBrushless);
    liftDrive = new DifferentialDrive(left, right);
    
  }

  @Override
  public void robotPeriodic() {
  }

  
  @Override
  public void autonomousInit() {
    
  }

  
  @Override
  public void autonomousPeriodic() {
    
  }


  @Override
  public void teleopPeriodic() {
    liftDrive.tankDrive(xbox.getY(Hand.kRight), xbox.getY(Hand.kRight));
  }
  


  @Override
  public void testPeriodic() {
  }
}
