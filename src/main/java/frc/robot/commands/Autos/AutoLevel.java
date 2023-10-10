package frc.robot.commands.Autos;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class AutoLevel extends CommandBase{
    private DriveSubsystem mSwerveDrive;
    private PIDController mPitchController;
    private PIDController mRollController;
    private boolean prevCentricState;
    private BooleanConsumer setFieldCentric;
    private double kP = 0.055;
    private double kD = 0;
    
    public AutoLevel(DriveSubsystem swerveDrive, BooleanSupplier fieldCentric, BooleanConsumer setFieldCentric){
        this.mSwerveDrive = swerveDrive;
        this.setFieldCentric = setFieldCentric;
        mPitchController = new PIDController(kP, 0.0, kD); // TODO: Dont hard code this in the future
        mRollController = new PIDController(kP, 0.0, kD);
        prevCentricState = fieldCentric.getAsBoolean();
        addRequirements(mSwerveDrive);
    }

    @Override
    public void execute() {
        setFieldCentric.accept(false);
        double pitchOutput = mPitchController.calculate(mSwerveDrive.getGyro().getPitch(), 0.0);
        double rollOutput = mRollController.calculate(mSwerveDrive.getGyro().getRoll(), 0.0);
        if(Math.abs(mSwerveDrive.getGyro().getPitch()) < 1 && Math.abs(mSwerveDrive.getGyro().getRoll()) < 1){
            pitchOutput = 0;
            rollOutput = 0;
            SmartDashboard.putBoolean("Level", true);
        } else             SmartDashboard.putBoolean("Level", false);

        mSwerveDrive.drive(
            new ChassisSpeeds(
                -Math.signum(pitchOutput) * pitchOutput * pitchOutput,
                Math.signum(rollOutput) * rollOutput * rollOutput,
                0.0
            )
        );

    }
    @Override
    public void end(boolean interrupted) {
        setFieldCentric.accept(prevCentricState);
    }


    @Override
    public boolean isFinished() {
        return false;
    }
    
}
