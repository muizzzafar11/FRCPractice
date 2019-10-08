/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.DriveSubsystem;

public class DriveCommandSpark extends Command {

  Spark leftMasteSpark = new Spark(RobotMap.LeftMasterSpark);
  Spark leftSlaveSpark = new Spark(RobotMap.LeftSlaveSpark);
  Spark RightMasteSpark = new Spark(RobotMap.rightMasterSpark);
  Spark RightSlaveSpark = new Spark(RobotMap.rightSlaveSpark);

  SpeedControllerGroup leftSpark = new SpeedControllerGroup(leftMasteSpark, leftSlaveSpark);
  SpeedControllerGroup rightSpark = new SpeedControllerGroup(RightMasteSpark, RightSlaveSpark);

  DifferentialDrive sparkDrive = new DifferentialDrive(leftSpark, rightSpark);

  public DriveCommandSpark() {
    // Use requires() here to declare subsystem dependencies
    requires(new DriveSubsystem());
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    sparkDrive.arcadeDrive(-OI.stick.getRawAxis(1) * Robot.Direction, OI.stick.getRawAxis(0));

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
