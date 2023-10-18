package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.configs.SuperStructureConfig;
import frc.robot.subsystems.Rotation;

public class RotationPID extends CommandBase{

    public enum RotationSetpoint {
        RESET(150),
        REST(175),
        FLOOR(50), //for teleop
        FLOORCONE(70),
        FLOORCONEKNOCK(65),
        HIGH(125),
        MID(116),
        MIDCONE(98),
        FLIPOVER(270),
        SUBSTATION(140),
        SINGLE(98);
    
        public double degrees;
    
        private RotationSetpoint(double degrees) {
            this.degrees = degrees;
        }
    
    }

    private Rotation mRotation;
    private double setpoint;
    private PIDController pidController;
    private double kP;
    private double kPG; // proportional gain against gravity
    private DoubleSupplier kMinAngle;
    private DoubleSupplier kMaxAngle;
    private DoubleSupplier kExtensionlength;
    private double kRotationDelta;
    private double kRotationDeltaProportional;
    private double kMaxExtension;

    public RotationPID(Rotation rotation,
            DoubleSupplier extensionLength,
            DoubleSupplier minAngle, 
            DoubleSupplier maxAngle,
            SuperStructureConfig config){
        this.mRotation = rotation; 
        this.setpoint = config.getRotationInitSetpoint();
        this.pidController = config.getRotationPIDController();
        this.kP = config.getRotationKP();
        this.kPG = config.getRotationKPG();
        this.kExtensionlength = extensionLength; 
        this.kMinAngle = minAngle; 
        this.kMaxAngle = maxAngle; 
        this.kRotationDelta = config.getRotationDelta();
        this.kRotationDeltaProportional = config.getRotationDeltaProportional();
        this.kMaxExtension = config.getMaxExtension();
        addRequirements(rotation);
    }

    public void setSetpoint(RotationSetpoint newSetpoint) {
        setpoint = newSetpoint.degrees;
    }

    public boolean atSetpoint(){
        return Math.abs(setpoint - mRotation.getRotationDegrees()) < 1; 
    }
    

    @Override
    public void execute() {
        double adjustedSetpoint = MathUtil.clamp(setpoint, kMinAngle.getAsDouble(), kMaxAngle.getAsDouble());

        pidController.setP(kPG - ((kP * kExtensionlength.getAsDouble()) / kMaxExtension));

        double output = Math.sin(mRotation.getRotationRadians()) * (kRotationDelta + (kRotationDeltaProportional * kExtensionlength.getAsDouble() / kMaxExtension)) 
        + pidController.calculate(mRotation.getRotationDegrees(), adjustedSetpoint);

        mRotation.set(output);


    }
    
}
