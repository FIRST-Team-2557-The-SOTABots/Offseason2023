package frc.robot.configs;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public class SuperStructureConfig {
    private double encoderAtZeroDegrees;
    private double encoderPerDegree;
    private int armBaseLength;
    private double encoderPerInch;
    private double height;
    private double backOffset;
    private double frontOffset;
    private double frontAbsoluteOffset;
    private double backAbsoluteOffset;

    public double getBackOffset() {
        return this.backOffset;
    }

    public void setBackOffset(double backOffset) {
        this.backOffset = backOffset;
    }

    public double getFrontOffset() {
        return this.frontOffset;
    }

    public void setFrontOffset(double frontOffset) {
        this.frontOffset = frontOffset;
    }

    public double getFrontAbsoluteOffset() {
        return this.frontAbsoluteOffset;
    }

    public void setFrontAbsoluteOffset(double frontAbsoluteOffset) {
        this.frontAbsoluteOffset = frontAbsoluteOffset;
    }

    public double getBackAbsoluteOffset() {
        return this.backAbsoluteOffset;
    }

    public void setBackAbsoluteOffset(double backAbsoluteOffset) {
        this.backAbsoluteOffset = backAbsoluteOffset;
    }

    private double maxExtension;
    private double roatationEncoderOffset;
    private double rotationDelta;
    private double rotationDeltaProportional;

    private double rotationInitSetpoint;

    private double rotationKPG;
    private double rotationKP;
    private double rotationKI;
    private double rotationKD;
    private double rotationMaxVel;
    private double rotationMaxAccel;

    private double extensionKP;
    private double extensionKI;
    private double extensionKD;
    private double extensionMaxVel;
    private double extensionMaxAccel;

    public double getEncoderAtZeroDegrees() {
        return this.encoderAtZeroDegrees;
    }

    public void setEncoderAtZeroDegrees(double encoderAtZeroDegrees) {
        this.encoderAtZeroDegrees = encoderAtZeroDegrees;
    }

    public double getEncoderPerDegree() {
        return this.encoderPerDegree;
    }

    public void setEncoderPerDegree(double encoderPerDegree) {
        this.encoderPerDegree = encoderPerDegree;
    }

    public int getArmBaseLength() {
        return this.armBaseLength;
    }

    public void setArmBaseLength(int armBaseLength) {
        this.armBaseLength = armBaseLength;
    }

    public double getEncoderPerInch() {
        return this.encoderPerInch;
    }

    public void setEncoderPerInch(double encoderPerInch) {
        this.encoderPerInch = encoderPerInch;
    }

    public double getHeight() {
        return this.height;
    }

    public void setHeight(double height) {
        this.height = height;
    }

    
    public double getMaxExtension() {
        return this.maxExtension;
    }

    public void setMaxExtension(double maxExtension) {
        this.maxExtension = maxExtension;
    }

    public double getRoatationEncoderOffset() {
        return this.roatationEncoderOffset;
    }

    public void setRoatationEncoderOffset(double roatationEncoderOffset) {
        this.roatationEncoderOffset = roatationEncoderOffset;
    }

    public double getRotationDelta() {
        return this.rotationDelta;
    }

    public void setRotationDelta(double rotationDelta) {
        this.rotationDelta = rotationDelta;
    }

    public double getRotationDeltaProportional() {
        return this.rotationDeltaProportional;
    }

    public void setRotationDeltaProportional(double rotationDeltaProportional) {
        this.rotationDeltaProportional = rotationDeltaProportional;
    }

    public double getRotationInitSetpoint() {
        return this.rotationInitSetpoint;
    }

    public void setRotationInitSetpoint(double rotationInitSetpoint) {
        this.rotationInitSetpoint = rotationInitSetpoint;
    }

    public double getRotationKPG() {
        return this.rotationKPG;
    }

    public void setRotationKPG(double rotationKPG) {
        this.rotationKPG = rotationKPG;
    }

    public double getRotationKP() {
        return this.rotationKP;
    }

    public void setRotationKP(double rotationKP) {
        this.rotationKP = rotationKP;
    }

    public double getRotationKI() {
        return this.rotationKI;
    }

    public void setRotationKI(double rotationKI) {
        this.rotationKI = rotationKI;
    }

    public double getRotationKD() {
        return this.rotationKD;
    }

    public void setRotationKD(double rotationKD) {
        this.rotationKD = rotationKD;
    }

    public double getRotationMaxVel() {
        return this.rotationMaxVel;
    }

    public void setRotationMaxVel(double rotationMaxVel) {
        this.rotationMaxVel = rotationMaxVel;
    }

    public double getRotationMaxAccel() {
        return this.rotationMaxAccel;
    }

    public void setRotationMaxAccel(double rotationMaxAccel) {
        this.rotationMaxAccel = rotationMaxAccel;
    }

    public double getExtensionKP() {
        return this.extensionKP;
    }

    public void setExtensionKP(double extensionKP) {
        this.extensionKP = extensionKP;
    }

    public double getExtensionKI() {
        return this.extensionKI;
    }

    public void setExtensionKI(double extensionKI) {
        this.extensionKI = extensionKI;
    }

    public double getExtensionKD() {
        return this.extensionKD;
    }

    public void setExtensionKD(double extensionKD) {
        this.extensionKD = extensionKD;
    }

    public double getExtensionMaxVel() {
        return this.extensionMaxVel;
    }

    public void setExtensionMaxVel(double extensionMaxVel) {
        this.extensionMaxVel = extensionMaxVel;
    }

    public double getExtensionMaxAccel() {
        return this.extensionMaxAccel;
    }

    public void setExtensionMaxAccel(double extensionMaxAccel) {
        this.extensionMaxAccel = extensionMaxAccel;
    }

    

    public PIDController getRotationPIDController() {
        PIDController pid = new PIDController(rotationKP, rotationKI, rotationKD);
        return pid;
    }

    public ProfiledPIDController getRotationProfiledPIDController() {
        ProfiledPIDController pid = new ProfiledPIDController(rotationKP, rotationKI, rotationKD, 
            new Constraints(rotationMaxVel, rotationMaxAccel));
        return pid;
    }

    public PIDController getExtensionPIDController() {
        PIDController pid = new PIDController(extensionKP, extensionKI, extensionKD);
        return pid;
    }

    public ProfiledPIDController getExtensionProfiledPIDController() {
        ProfiledPIDController pid = new ProfiledPIDController(extensionKP, extensionKI, extensionKD, 
            new Constraints(extensionMaxVel, extensionMaxAccel));
        return pid;
    }


}

