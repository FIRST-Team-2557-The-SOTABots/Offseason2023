// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;


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
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.ExtensionPID;
import frc.robot.commands.JonasFunkyIntake;
import frc.robot.commands.ResetExtension;
import frc.robot.commands.RotationPID;
import frc.robot.commands.Autos.AutoLevel;
import frc.robot.commands.Autos.ConeMobilBump;
import frc.robot.commands.Autos.ConeMobilCharge;
import frc.robot.commands.Autos.PlaceCone;
import frc.robot.commands.ExtensionPID.ExtensionSetpoint;
import frc.robot.commands.RotationPID.RotationSetpoint;
import frc.robot.configs.SuperStructureConfig;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Extension;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Rotation;
import frc.robot.subsystems.SuperStructure;
import frc.robot.commands.Autos.PlaceConeCharge;
import frc.robot.commands.Autos.PlaceConeMobil;
import frc.robot.commands.Autos.PlaceCubeCharge;
import frc.robot.commands.Autos.PlaceCubeMobil;
import frc.robot.commands.Autos.TestAuto;
import SOTAlib.Factories.AutoFactory;


public class RobotContainer {

  private ConfigUtils configUtils;
  private SOTA_Xboxcontroller dController = new SOTA_Xboxcontroller(0);
  private SOTA_Xboxcontroller mController = new SOTA_Xboxcontroller(1);
  private final DriveSubsystem mDriveTrain;
  private boolean driveFieldCentric = true; // REV code is trash and we have to manage this out here
  private boolean driveLowGear = false; //not a physical low gear just an in code thing
  private Extension mExtension;
  private Rotation mRotation;
  private Intake mIntake;
  private SuperStructure superStructure;
  private RotationPID rotationPID;
  private ExtensionPID extensionPID;
  private ResetExtension mResetExtension;
  private JonasFunkyIntake mFunkyIntake;
  private SwerveAutoBuilder mAutoBuilder;
  private SendableChooser<Command> mAutoChooser;


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

      SOTA_MotorController intakeMotor1 = MotorControllerFactory
          .generateMotorController(configUtils.readFromClassPath(MotorControllerConfig.class,
              "SuperStructure/IntakeMotor1"));
      SOTA_MotorController intakeMotor2 = MotorControllerFactory
          .generateMotorController(configUtils.readFromClassPath(MotorControllerConfig.class,
              "SuperStructure/IntakeMotor2"));

      DigitalInput limitSwitch = new DigitalInput(0);

      SuperStructureConfig superStructureConfig = configUtils.readFromClassPath(SuperStructureConfig.class,
          "SuperStructure/SuperStructure");

      this.mExtension = new Extension(winchMotor, limitSwitch, superStructureConfig);
      this.mRotation = new Rotation(rotationMotor, rotatiEncoder, superStructureConfig);
      this.mIntake = new Intake(intakeMotor1, intakeMotor2);

      this.superStructure = new SuperStructure(mExtension::getLength, mRotation::getRotationDegrees,
          superStructureConfig);

      ProfiledPIDController extensController = new ProfiledPIDController(5, 0, 0,
          new TrapezoidProfile.Constraints(20.0, 20.0));

      this.rotationPID = new RotationPID(mRotation, mExtension::getLengthFromStart, superStructure::minRotation,
          superStructure::maxRotation, superStructureConfig);
      this.extensionPID = new ExtensionPID(extensController, mExtension,
          superStructure::maxExtension);
      this.mResetExtension = new ResetExtension(mExtension);

    } catch (Exception e) {
      e.printStackTrace();
    }

    Map<String, Command> eventMap = new HashMap<String, Command>();

          eventMap.put("test-event", new PrintCommand("test 1"));

          eventMap.put("test-event2", new PrintCommand("teset 2"));

          
      // eventMap.put("test-event", new InstantCommand(
      //   () -> {
      //     rotationPID.setSetpoint(RotationSetpoint.MID);
      //     extensionPID.setSetpoint(ExtensionSetpoint.MID);
      //   },
      //   mRotation, mExtension
      // ));
      // eventMap.put("event2", new InstantCommand(
      //   () -> {
      //     rotationPID.setSetpoint(RotationSetpoint.MID);
      //     extensionPID.setSetpoint(ExtensionSetpoint.MID);
      //   },
      //   mRotation, mExtension
      // ));
      mAutoBuilder = AutoFactory.swerveAutoBuilderGenerator(mDriveTrain, eventMap);

    configureAutos();

    setDefaultCommands();
    configureBindings();
  }

  private void setDefaultCommands() {
    mDriveTrain.setDefaultCommand(new DriveCommand(dController::getLeftY, dController::getLeftX, dController::getRightX,
        () -> getFieldCentric(), () -> isLowGear(), mDriveTrain));
    
    mRotation.setDefaultCommand(rotationPID); // ???
    mExtension.setDefaultCommand(extensionPID);
  }

  private boolean isLowGear() {
    return driveLowGear;
  }

  private Boolean getFieldCentric() {
    return driveFieldCentric;
  }

  private void setFieldCentric(boolean state) {
    this.driveFieldCentric = state;
  }

  private void configureBindings() {
    dController.start().onTrue(new InstantCommand(
        () -> mDriveTrain.zeroHeading(), mDriveTrain));

    dController.back().whileTrue(new AutoLevel(mDriveTrain, 
    () -> getFieldCentric(), 
    (state) -> setFieldCentric(state)));

    dController.rightBumper().onTrue(new InstantCommand(() -> {
      driveFieldCentric = false;
      SmartDashboard.putBoolean("fieldCentric", driveFieldCentric);
    }, mDriveTrain));

    dController.leftBumper().onTrue(new InstantCommand(() -> {
      driveFieldCentric = true;
      SmartDashboard.putBoolean("fieldCentric", driveFieldCentric);
    }, mDriveTrain));

    dController.getRightTrigger().onTrue(new InstantCommand(() -> {
      driveLowGear = true;
      SmartDashboard.putBoolean("Low Gear", driveLowGear);
    }, mDriveTrain)).onFalse(new InstantCommand(() -> {
      driveLowGear = false;
      SmartDashboard.putBoolean("Low Gear", driveLowGear);
    }, mDriveTrain));

    mController.a().onTrue(new InstantCommand(() -> {
      rotationPID.setSetpoint(RotationSetpoint.FLOOR);
      extensionPID.setSetpoint(ExtensionSetpoint.FLOOR);
    }, mRotation, mExtension)).onFalse(restCommand());

    mController.b().onTrue(new InstantCommand(() -> {
      rotationPID.setSetpoint(RotationSetpoint.MID);
      extensionPID.setSetpoint(ExtensionSetpoint.MID);
    }, mRotation, mExtension)).onFalse(restCommand());

    mController.x().onTrue(new InstantCommand(() -> {
    rotationPID.setSetpoint(RotationSetpoint.MIDCONE);
    extensionPID.setSetpoint(ExtensionSetpoint.MIDCONE);
    }, mRotation, mExtension)).onFalse(restCommand());

    mController.y().onTrue(new InstantCommand(() -> {
      rotationPID.setSetpoint(RotationSetpoint.HIGH);
      extensionPID.setSetpoint(ExtensionSetpoint.HIGH);
    }, mRotation, mExtension)).onFalse(restCommand());

    mController.getRightTrigger().onTrue(new InstantCommand(() -> {
      rotationPID.setSetpoint(RotationSetpoint.SINGLE);
      extensionPID.setSetpoint(ExtensionSetpoint.SINGLE);
    }, mRotation, mExtension)).onFalse(restCommand());

    mController.getLeftTrigger().onTrue(new InstantCommand(() -> {
      rotationPID.setSetpoint(RotationSetpoint.SUBSTATION);
      extensionPID.setSetpoint(ExtensionSetpoint.SUBSTATION);
    }, mRotation, mExtension)).onFalse(restCommand());

    mController.start().onTrue(mResetExtension);

    mController.leftBumper().whileTrue(Commands.run(
        () -> mIntake.set(0.5), mIntake)).onFalse(Commands.run(() -> mIntake.set(0.0), mIntake));

    mController.rightBumper().whileTrue(Commands.run(() -> mIntake.set(-0.5), mIntake))
        .onFalse(Commands.run(() -> mIntake.set(0), mIntake));
  }

  

  public void configureAutos(){

    this.mAutoChooser = new SendableChooser<>();
    this.mAutoChooser.setDefaultOption("None", null);

    // List of autos to choose from

    // mAutoChooser.addOption("Place and Mobility", new PlaceConeMobility(mDriveTrain, getNewExtensionPID(), getNewRotationPID(), mAutoBuilder, 
    //   mIntake, PathPlanner.loadPath("R Cone Mobil", new PathConstraints(2, 1)), new ResetExtension(mExtension)));
    // mAutoChooser.addOption("Place Charge Mobility", new PlaceConeCharge(getNewExtensionPID(), getNewRotationPID(), mIntake, mAutoBuilder, mDriveTrain, 
    // mResetExtension, PathPlanner.loadPath("C Cone Mobil Charge", new PathConstraints(1, 1))));
    // mAutoChooser.addOption("Test path", new TestAuto(mDriveTrain, getNewExtensionPID(), getNewRotationPID(), mAutoBuilder, mIntake, 
    // PathPlanner.loadPath("Test Path", new PathConstraints(2, 2)), new ResetExtension(mExtension)));
    mAutoChooser.addOption("Place Cone", new PlaceCone(mDriveTrain, getNewExtensionPID(), getNewRotationPID(), mIntake, new ResetExtension(mExtension)));
    // mAutoChooser.addOption("Place Cube", new PrintCommand("not done")); //TODO: make this
    mAutoChooser.addOption("Place Cone Mobility", new PlaceConeMobil(mDriveTrain, getNewExtensionPID(), new ResetExtension(mExtension), 
    new AutoLevel(mDriveTrain, () -> getFieldCentric(), (state) -> setFieldCentric(state)), getNewRotationPID(), mIntake));
    // mAutoChooser.addOption("Place Cube Mobility", new PlaceCubeMobil(mDriveTrain, getNewExtensionPID(), new ResetExtension(mExtension),
    // new AutoLevel(mDriveTrain, () -> getFieldCentric(), (state) -> setFieldCentric(state)), getNewRotationPID(), mIntake));
    mAutoChooser.addOption("Place Cone Charge", new PlaceConeCharge(mDriveTrain, getNewExtensionPID(), new ResetExtension(mExtension),
    new AutoLevel(mDriveTrain, () -> getFieldCentric(), (state) -> setFieldCentric(state)), getNewRotationPID(), mIntake));
    // mAutoChooser.addOption("Place Cube Charge", new PlaceCubeCharge(mDriveTrain, getNewExtensionPID(), new ResetExtension(mExtension), 
    // new AutoLevel(mDriveTrain, () -> getFieldCentric(),(state) -> setFieldCentric(state)), rotationPID, mIntake));
    mAutoChooser.addOption("Place Cone Mobil Bump", new ConeMobilBump(mDriveTrain, getNewExtensionPID(), new ResetExtension(mExtension), getNewRotationPID(),mIntake));
    // mAutoChooser.addOption("Place Cube Mobil Bump", new PrintCommand("not done")); //TODO: make this
    mAutoChooser.addOption("Cone Mobil Charge", new ConeMobilCharge(mDriveTrain, getNewExtensionPID(), new ResetExtension(mExtension), 
    new AutoLevel(mDriveTrain, () -> getFieldCentric(), (state) -> setFieldCentric(state)), getNewRotationPID(), mIntake));

    SmartDashboard.putData(mAutoChooser);
  }

  public ExtensionPID getNewExtensionPID() {

    ProfiledPIDController autoExtensController = new ProfiledPIDController(3, 0, 0, new TrapezoidProfile.Constraints(40.8, 80.0));
    return new ExtensionPID(autoExtensController, mExtension, superStructure::maxExtension);

  }

  public RotationPID getNewRotationPID(){
    try {
      SuperStructureConfig autoSuperStructureConfig = configUtils.readFromClassPath(SuperStructureConfig.class,"SuperStructure/SuperStructure");
    return new RotationPID(mRotation, mExtension::getLengthFromStart, 
    superStructure::minRotation, superStructure::maxRotation, autoSuperStructureConfig);
    } catch(IOException e) {
      throw new RuntimeException("Not able to create rotation PID");
    }
  }

  public InstantCommand restCommand() {
    return new InstantCommand(() -> {
      rotationPID.setSetpoint(RotationSetpoint.REST);
      extensionPID.setSetpoint(ExtensionSetpoint.REST);
    }, mRotation, mExtension);
  }

  public Command getAutonomousCommand() {
    return mAutoChooser.getSelected();
    // return new PrintCommand("No Autonomous Configured");
  }

}
