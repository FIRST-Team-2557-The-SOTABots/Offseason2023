package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import javax.swing.text.StyleContext.SmallAttributeSet;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveCommand extends CommandBase {
    private DoubleSupplier frwdSup;
    private DoubleSupplier strfSup;
    private DoubleSupplier rotSup;
    private DriveSubsystem mDrive;
    private BooleanSupplier fieldCentric;
    private BooleanSupplier lowGear;

    public DriveCommand(DoubleSupplier frwdSup, DoubleSupplier strfSup, DoubleSupplier rotSup,
            BooleanSupplier fieldCentric, BooleanSupplier lowGear, DriveSubsystem drive) {
        this.frwdSup = frwdSup;
        this.strfSup = strfSup;
        this.rotSup = rotSup;
        this.fieldCentric = fieldCentric;
        this.lowGear = lowGear;
        this.mDrive = drive;

        addRequirements(mDrive);
    }

    @Override
    public void execute() {
        double frwd;
        double strf;
        double rot;

        SmartDashboard.putBoolean("lowGear", lowGear.getAsBoolean());

        if (lowGear.getAsBoolean()) {
            frwd = frwdSup.getAsDouble() * 0.5;
            strf = strfSup.getAsDouble() * 0.5;
            rot = rotSup.getAsDouble() * 0.5;
        } else {
            frwd = frwdSup.getAsDouble();
            strf = strfSup.getAsDouble();
            rot = rotSup.getAsDouble();
        }
        mDrive.drive(
                -MathUtil.applyDeadband(frwd, OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(strf, OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(rot, OIConstants.kDriveDeadband),
                fieldCentric.getAsBoolean(), false);
    }

}
