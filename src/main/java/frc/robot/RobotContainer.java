// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

// import com.pathplanner.lib.auto.SwerveAutoBuilder;
// import com.pathplanner.lib.PathPlanner;

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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
import frc.robot.commands.Autos.CubeMobilBump;
import frc.robot.commands.Autos.IntakeCubeGround;
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
import frc.robot.commands.Autos.PlaceHighCube;
import frc.robot.commands.Autos.TestAuto;
// import SOTAlib.Factories.AutoFactory;

public class RobotContainer {

  private ConfigUtils configUtils;
  private SOTA_Xboxcontroller dController = new SOTA_Xboxcontroller(0);
  private SOTA_Xboxcontroller mController = new SOTA_Xboxcontroller(1);
  private final DriveSubsystem mDriveTrain;
  private boolean driveFieldCentric = true; // REV code is trash and we have to manage this out here
  private boolean driveLowGear = false; // not a physical low gear just an in code thing
  private Extension mExtension;
  private Rotation mRotation;
  private Intake mIntake;
  private SuperStructure superStructure;
  private RotationPID rotationPID;
  private ExtensionPID extensionPID;
  private ResetExtension mResetExtension;
  private JonasFunkyIntake mFunkyIntake;
  private SendableChooser<Command> mAutoChooser;
  private SendableChooser<String> mAutoChooser2;

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


   
    registerCommands();
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

    Command[] highRest = {new InstantCommand(() -> extensionPID.setSetpoint(ExtensionSetpoint.REST), mExtension),
      new SequentialCommandGroup(new WaitCommand(2), new InstantCommand(() -> rotationPID.setSetpoint(RotationSetpoint.REST), mRotation))};

    mController.y().onTrue(new InstantCommand(() -> {
      rotationPID.setSetpoint(RotationSetpoint.HIGH);
      extensionPID.setSetpoint(ExtensionSetpoint.HIGH);
    }, mRotation, mExtension)).onFalse(restCommand());//.onFalse(new SequentialCommandGroup(new InstantCommand(() -> extensionPID.setSetpoint(ExtensionSetpoint.REST), mExtension).withTimeout(1), new InstantCommand(() -> rotationPID.setSetpoint(RotationSetpoint.REST), mRotation)));
    
  

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

    mController.back().onTrue(new InstantCommand(() -> {
      rotationPID.setSetpoint(RotationSetpoint.FLIPOVER);
    }, mRotation)).onFalse(restCommand());
  }

  public void registerCommands() {

    NamedCommands.registerCommand("autoBal", new AutoLevel(mDriveTrain, () -> getFieldCentric(), (state) -> setFieldCentric(state)));
    NamedCommands.registerCommand("PlaceHighCone", new PlaceCone(mDriveTrain, getNewExtensionPID(), getNewRotationPID(), mIntake, new ResetExtension(mExtension)));
    NamedCommands.registerCommand("IntakeCubeGnd", new IntakeCubeGround(mDriveTrain, getNewExtensionPID(), getNewRotationPID(), mIntake, new ResetExtension(mExtension)));
    NamedCommands.registerCommand("PlacecHighCube", new PlaceHighCube(mDriveTrain, getNewExtensionPID(), getNewRotationPID(), mIntake, new ResetExtension(mExtension)));
    NamedCommands.registerCommand("resetExtension", new ResetExtension(mExtension));

    // NamedCommands.registerCommand("test1", new testCommand1());

  }

  public void configureAutos() {

    this.mAutoChooser = new SendableChooser<>();
    this.mAutoChooser.setDefaultOption("None", null);

    // List of autos to choose from

   
    mAutoChooser.addOption("Cone",
        new PlaceCone(mDriveTrain, getNewExtensionPID(), getNewRotationPID(), mIntake, new ResetExtension(mExtension)));
    mAutoChooser.addOption("Cone Mobil",
        new PlaceConeMobil(mDriveTrain, getNewExtensionPID(), new ResetExtension(mExtension),
            new AutoLevel(mDriveTrain, () -> getFieldCentric(), (state) -> setFieldCentric(state)), getNewRotationPID(), mIntake));
    mAutoChooser.addOption("Cone Charge",
        new PlaceConeCharge(mDriveTrain, getNewExtensionPID(), new ResetExtension(mExtension),
            new AutoLevel(mDriveTrain, () -> getFieldCentric(), (state) -> setFieldCentric(state)), getNewRotationPID(), mIntake));
    mAutoChooser.addOption("Cone Mobil Bump", new ConeMobilBump(mDriveTrain, getNewExtensionPID(),
        new ResetExtension(mExtension), getNewRotationPID(), mIntake));
    mAutoChooser.addOption("Cone Mobil Charge",
        new ConeMobilCharge(mDriveTrain, getNewExtensionPID(), new ResetExtension(mExtension),
            new AutoLevel(mDriveTrain, () -> getFieldCentric(), (state) -> setFieldCentric(state)), getNewRotationPID(), mIntake));

    this.mAutoChooser2 = new SendableChooser<>();
    this.mAutoChooser2.setDefaultOption("None", null);

    mAutoChooser2.addOption("Place Cone", null);
    mAutoChooser2.addOption("Test 1", "Test Path 1");
    mAutoChooser2.addOption("test 2", "Test Path 2");
    mAutoChooser2.addOption("teset 3", "Test Path 3");
    mAutoChooser2.addOption("test 4", "Test 4 (event)");

    SmartDashboard.putData("DONT USE", mAutoChooser); // use this selector for command based autos. getAutonomousCommand will need to be changed to use this
    SmartDashboard.putData("Pathplanner Autos", mAutoChooser2); // use this selector for pathplanner based autos. getAutonomousCommand will also need to be changed
  }

  public ExtensionPID getNewExtensionPID() {

    ProfiledPIDController autoExtensController = new ProfiledPIDController(3, 0, 0,
        new TrapezoidProfile.Constraints(40.8, 80.0));
    return new ExtensionPID(autoExtensController, mExtension, superStructure::maxExtension);

  }

  public RotationPID getNewRotationPID() {
    try {
      SuperStructureConfig autoSuperStructureConfig = configUtils.readFromClassPath(SuperStructureConfig.class,
          "SuperStructure/SuperStructure");
      return new RotationPID(mRotation, mExtension::getLengthFromStart,
          superStructure::minRotation, superStructure::maxRotation, autoSuperStructureConfig);
    } catch (IOException e) {
      throw new RuntimeException("Not able to create rotation PID");
    }
  }

  public InstantCommand restCommand() {
    return new InstantCommand(() -> {
      rotationPID.setSetpoint(RotationSetpoint.REST);
      extensionPID.setSetpoint(ExtensionSetpoint.REST);
    }, mRotation, mExtension);
  }

  // public Command getAutonomousCommand() {
  //   return mAutoChooser.getSelected();
  // }

  public Command getAutonomousCommand(){

    // FOR COMMAND BASED AUTOS:

    
    // return mAutoChooser.getSelected();    



    // FOR PATHPLANNER AUTOS:


    // Load the path you want to follow using its name in the GUI
    PathPlannerPath path = PathPlannerPath.fromPathFile(mAutoChooser2.getSelected());

    // Create a path following command using AutoBuilder. This will also trigger event markers.
    return AutoBuilder.followPathWithEvents(path);
}

}
