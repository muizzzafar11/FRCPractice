/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotMap;
// import frc.robot.subsystems.DriveSubsystem;

public class DriveCommand extends Command {


  static WPI_TalonSRX leftMaster = new WPI_TalonSRX(RobotMap.LeftMaster);
  static WPI_TalonSRX leftSlave = new WPI_TalonSRX(RobotMap.LeftSlave);
  static WPI_TalonSRX rightMaster = new WPI_TalonSRX(RobotMap.rightMaster);
  static WPI_TalonSRX rightSlave = new WPI_TalonSRX(RobotMap.rightSlave);

  static SpeedControllerGroup left = new SpeedControllerGroup(leftMaster, leftSlave);
  static SpeedControllerGroup right = new SpeedControllerGroup(rightMaster, rightSlave);

  public static DifferentialDrive drive = new DifferentialDrive(left, right);

  public DriveCommand() {
    // Use requires() here to declare subsystem dependencies
    
      requires(Robot.driveSubsystem);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  drive.arcadeDrive(-OI.stick.getRawAxis(1),OI.stick.getRawAxis(0));
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
