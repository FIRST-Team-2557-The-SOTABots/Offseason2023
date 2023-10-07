// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import SOTAlib.Config.ConfigUtils;
import SOTAlib.Config.EncoderConfig;
import SOTAlib.Config.MotorControllerConfig;
import SOTAlib.Control.SOTA_Xboxcontroller;
import SOTAlib.Encoder.Absolute.SOTA_AbsoulteEncoder;
import SOTAlib.Factories.EncoderFactory;
import SOTAlib.Factories.MotorControllerFactory;
import SOTAlib.MotorController.SOTA_MotorController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.RotationPID;
import frc.robot.commands.RotationPID.RotationSetpoint;
import frc.robot.configs.SuperStructureConfig;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Extension;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Rotation;
import frc.robot.subsystems.SuperStructure;

public class RobotContainer {

  private ConfigUtils configUtils;
  private SOTA_Xboxcontroller dController = new SOTA_Xboxcontroller(0);
  private SOTA_Xboxcontroller mController = new SOTA_Xboxcontroller(1);
  private final DriveSubsystem mDriveTrain;
  private boolean driveFieldCentric = true; // REV code is trash and we have to manage this out here
  private Extension mExtension;
  private Rotation mRotation;
  private Intake mIntake;
  private SuperStructure superStructure;
  private RotationPID rotationPID;
  // private ExtensionPID extensionPID;

  public RobotContainer() {
    this.configUtils = new ConfigUtils();
    this.mDriveTrain = new DriveSubsystem();
    SmartDashboard.putBoolean("FieldCentric", driveFieldCentric);

    try {
      SOTA_MotorController rotationMotor = MotorControllerFactory.generateMotorController(
          (configUtils.readFromClassPath(MotorControllerConfig.class, "SuperStructure/RotatorMotor")));

      SOTA_AbsoulteEncoder rotatiEncoder = EncoderFactory.generateAbsoluteEncoder(
          configUtils.readFromClassPath(EncoderConfig.class, "SuperStructure/RotationEncoder"));

      SOTA_MotorController winchMotor = MotorControllerFactory.generateMotorController(
          configUtils.readFromClassPath(MotorControllerConfig.class, "SuperStructure/WinchMotor"));

      // SOTA_MotorController intakeMotorTop =
      // MotorControllerFactory.generateMotorController
      // (configUtils.readFromClassPath(MotorControllerConfig.class,
      // "SuperStructure/IntakeMotorTop"));
      // SOTA_MotorController intakeMotorBottom =
      // MotorControllerFactory.generateMotorController
      // (configUtils.readFromClassPath(MotorControllerConfig.class,
      // "SuperStructure/IntakeMotorBottom"));

      DigitalInput limitSwitch = new DigitalInput(0);

      SuperStructureConfig superStructureConfig = configUtils.readFromClassPath(SuperStructureConfig.class,
          "SuperStructure/SuperStructure");

      this.mExtension = new Extension(winchMotor, limitSwitch, superStructureConfig);
      this.mRotation = new Rotation(rotationMotor, rotatiEncoder, superStructureConfig);
      // this.mIntake = new Intake(intakeMotorTop, intakeMotorBottom);

      this.superStructure = new SuperStructure(mExtension::getLength, mRotation::getRotationDegrees,
          superStructureConfig);

      ProfiledPIDController extensController = new ProfiledPIDController(5, 0, 0,
          new TrapezoidProfile.Constraints(60.0, 100.0));

      this.rotationPID = new RotationPID(mRotation, mExtension::getLengthFromStart, superStructure::minRotation,
          superStructure::maxRotation, superStructureConfig);
      // this.extensionPID = new ExtensionPID(extensController, mExtension,
      // superStructure::maxExtension);
      // this.mResetExtension = new ResetExtension(mExtension);
      // this.intakeCommand = new BasicIntakeCommand(mIntake, mController::getLeftY);

    } catch (Exception e) {
      e.printStackTrace();
    }
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

    mRotation.setDefaultCommand(rotationPID);
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

    mController.a().onTrue(new InstantCommand(() -> rotationPID.setSetpoint(RotationSetpoint.SUBSTATION), mRotation));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
