// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import SOTAlib.Control.SOTA_Xboxcontroller;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;

public class RobotContainer {

  private SOTA_Xboxcontroller dController = new SOTA_Xboxcontroller(0);
  private final DriveSubsystem mDriveTrain;
  private boolean driveFieldCentric = true; // REV code is trash and we have to manage this out here

  public RobotContainer() {
    this.mDriveTrain = new DriveSubsystem();
    SmartDashboard.putBoolean("FieldCentric", driveFieldCentric);
    setDefaultCommands();
    configureBindings();
  }

  private void setDefaultCommands() {
    mDriveTrain.setDefaultCommand(new RunCommand(
        () -> mDriveTrain.drive(
            -MathUtil.applyDeadband(dController.getLeftY(), OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband(dController.getLeftX(), OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband(dController.getRightX(), OIConstants.kDriveDeadband),
            driveFieldCentric, false),
        mDriveTrain));
  }

  private void configureBindings() {
    dController.start().onTrue(new InstantCommand(
        () -> mDriveTrain.zeroHeading(), mDriveTrain));

    dController.y().onTrue(new InstantCommand(
        () -> {
          if (driveFieldCentric) {
            driveFieldCentric = false;
          } else {
            driveFieldCentric = true;
          }
          SmartDashboard.putBoolean("FieldCentric", driveFieldCentric);
        }, mDriveTrain));
  }


  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
