package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.configs.SuperStructureConfig;
import SOTAlib.Encoder.Absolute.SOTA_AbsoulteEncoder;
import SOTAlib.MotorController.SOTA_MotorController;

public class Rotation extends SubsystemBase {
  private SOTA_MotorController mMotor;
  private SOTA_AbsoulteEncoder mEncoder;
  private double kEncoderAtZero;
  private double kEncoderPerDegree;
  private ShuffleboardTab sTab;
  private GenericEntry absEncoderPosEntry;
  private GenericEntry relEncoderPosEntry;
  private GenericEntry armAngleDegreesEntry;
  private GenericEntry absEncoderNoOffsetEntry;

  public Rotation(SOTA_MotorController motor, SOTA_AbsoulteEncoder encoder, SuperStructureConfig config) {
    this.mMotor = motor;
    this.mEncoder = encoder;
    kEncoderAtZero = config.getEncoderAtZeroDegrees();
    kEncoderPerDegree = 1/360.0;

    this.sTab = Shuffleboard.getTab("Rotation");
    this.absEncoderPosEntry = sTab.add("ABS encoder position: ", 0.0).getEntry();
    this.relEncoderPosEntry = sTab.add("REL Encoer Position:", 0.0).getEntry();
    this.armAngleDegreesEntry = sTab.add("Arm Angle Deg:", 0.0).getEntry();
    this.absEncoderNoOffsetEntry = sTab.add("ABS NoOffset: ", 0.0).getEntry();
  }

  public void set(double speed) {
    mMotor.set(speed);
  }

  public double getRotatorEncoder() {
    return mEncoder.getPosition();
  }

  public double getRotationDegrees() {
    return (getRotatorEncoder()) / kEncoderPerDegree;
  }

  public double getRotationRadians() {
    return ((2 * Math.PI) / 360) * getRotationDegrees();
  }

  private void updateSD() {
    absEncoderPosEntry.setDouble(mEncoder.getPosition());
    relEncoderPosEntry.setDouble(mMotor.getEncoderPosition());
    armAngleDegreesEntry.setDouble(getRotationDegrees());
    absEncoderNoOffsetEntry.setDouble(mEncoder.getPositionNoOffset());

  }

  @Override
  public void periodic() {
    // mMotor.setEncoderPosition(mEncoder.getPosition());
    updateSD();
    // SmartDashboard.putNumber("Arm Angle", getRotationDegrees());
    // SmartDashboard.putNumber("Angle Encoder", getRotatorEncoder());
    // SmartDashboard.putNumber("No Offset",
    // mMotor.getEncoder().getAbsolutePosition());
    // // SmartDashboard.putNumber("Angle Encoder", getRotatorEncoder());
    // // SmartDashboard.putNumber("angle from native",
    // motor.getNativeEncoderPose());
    // SmartDashboard.putNumber("angle from native", mMotor.getEncoderPosition());
    // // SmartDashboard.putNumber("Rotation motor limit",
    // motor.getMotorLimits().getUpperLimit());
    // SmartDashboard.putNumber("Rotation Current", mMotor.getMotorCurrent());
  }
}
