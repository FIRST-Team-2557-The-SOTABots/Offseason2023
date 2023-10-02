// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;

import com.kauailabs.navx.frc.AHRS;

import SOTAlib.Config.ConfigUtils;
import SOTAlib.Config.SwerveDriveConfig;
import SOTAlib.Config.SwerveModuleConfig;
import SOTAlib.Control.SOTA_Xboxcontroller;
import SOTAlib.Factories.CompositeMotorFactory;
import SOTAlib.Factories.MotorControllerFactory;
import SOTAlib.Gyro.NavX;
import SOTAlib.MotorController.SOTA_CompositeMotor;
import SOTAlib.MotorController.SOTA_MotorController;
import SOTAlib.Swerve.SwerveDrive;
import SOTAlib.Swerve.SwerveDriveInterface;
import SOTAlib.Swerve.SwerveModule;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Commands.DriveCommand;

public class RobotContainer {
  private SOTA_Xboxcontroller dController;
  private SOTA_CompositeMotor tCompositeMotor;
  private SwerveDriveInterface mSwerveDrive;

  public RobotContainer() {
    CompositeMotorFactory mCompositeMotorFactory = new CompositeMotorFactory();
    ConfigUtils mConfigUtils = new ConfigUtils();
    
    dController = new SOTA_Xboxcontroller(0);

    //Swerve Creation
    try {
      //Configs
      SwerveDriveConfig driveConfig = mConfigUtils.readFromClassPath(SwerveDriveConfig.class, "Swerve/Drive");

      SwerveModuleConfig flConfig = mConfigUtils.readFromClassPath(SwerveModuleConfig.class, "Swerve/FrontLeft");
      SwerveModuleConfig frConfig = mConfigUtils.readFromClassPath(SwerveModuleConfig.class, "Swerve/FrontRight");
      SwerveModuleConfig blConfig = mConfigUtils.readFromClassPath(SwerveModuleConfig.class, "Swerve/BackLeft");
      SwerveModuleConfig brConfig = mConfigUtils.readFromClassPath(SwerveModuleConfig.class, "Swerve/BackRight");
      
      //Module Initializations
      SOTA_MotorController frontLeftSpeed = MotorControllerFactory.generateMotorController(flConfig.getSpeedConfig());
      SOTA_CompositeMotor frontLeftAngle = mCompositeMotorFactory.generateCompositeMotor(flConfig.getAngleConfig());
      SwerveModule frontLeftModule = new SwerveModule(frontLeftSpeed, frontLeftAngle.getMotor(), frontLeftAngle.getAbsEncoder(), flConfig, driveConfig);

      SOTA_MotorController frontRightSpeed = MotorControllerFactory.generateMotorController(frConfig.getSpeedConfig());
      SOTA_CompositeMotor frontRightAngle = mCompositeMotorFactory.generateCompositeMotor(frConfig.getAngleConfig());
      SwerveModule frontRightModule = new SwerveModule(frontRightSpeed, frontRightAngle.getMotor(), frontRightAngle.getAbsEncoder(), frConfig, driveConfig);

      SOTA_MotorController backLeftSpeed = MotorControllerFactory.generateMotorController(blConfig.getSpeedConfig());
      SOTA_CompositeMotor backLeftAngle = mCompositeMotorFactory.generateCompositeMotor(blConfig.getAngleConfig());
      SwerveModule backLeftModule = new SwerveModule(backLeftSpeed, backLeftAngle.getMotor(), backLeftAngle.getAbsEncoder(), blConfig, driveConfig);

      SOTA_MotorController backRightSpeed = MotorControllerFactory.generateMotorController(brConfig.getSpeedConfig());
      SOTA_CompositeMotor backRightAngle = mCompositeMotorFactory.generateCompositeMotor(brConfig.getAngleConfig());
      SwerveModule backRightModule = new SwerveModule(backRightSpeed, backRightAngle.getMotor(), backRightAngle.getAbsEncoder(), brConfig, driveConfig);

      SwerveModule[] modules = {frontLeftModule, frontRightModule, backLeftModule, backRightModule};

      SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        new Translation2d(driveConfig.getWheelBase()/2, -driveConfig.getTrackWidth()/2), //Front Left
        new Translation2d(driveConfig.getWheelBase()/2, driveConfig.getTrackWidth()/2), //Front Right
        new Translation2d(-driveConfig.getWheelBase()/2, -driveConfig.getTrackWidth()/2), //Back Left
        new Translation2d(-driveConfig.getWheelBase()/2, driveConfig.getTrackWidth()/2) //Back Right
      );
      NavX navX = new NavX(new AHRS(Port.kMXP));
      
      this.mSwerveDrive = new SwerveDrive(modules, navX, kinematics, driveConfig);

    }catch(IOException e) {
      System.out.println("CONFIGUTILS FAILURE");
      e.printStackTrace();
    } catch (Exception e) {
      System.out.println("Other failure");
      e.printStackTrace();
    }
    
    configureBindings();
    configureDefaultCommands();
    
  }

  private void configureDefaultCommands() {
    mSwerveDrive.setDefaultCommand(new DriveCommand(dController::getLeftY, dController::getLeftX, dController::getRightX, mSwerveDrive));
  }

  private void configureBindings() {
    dController.start().onTrue(new InstantCommand(() -> mSwerveDrive.resetGyro(), mSwerveDrive));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
