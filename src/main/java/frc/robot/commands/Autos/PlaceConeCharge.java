package frc.robot.commands.Autos;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import frc.robot.commands.ExtensionPID;
import frc.robot.commands.ResetExtension;
import frc.robot.commands.RotationPID;
import frc.robot.commands.ExtensionPID.ExtensionSetpoint;
import frc.robot.commands.RotationPID.RotationSetpoint;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Extension;

public class PlaceConeCharge extends SequentialCommandGroup {
    
    private Extension extension;

    public PlaceConeCharge(
        ExtensionPID extensionPID,
        RotationPID rotationPID,
        Intake intake,
        SwerveAutoBuilder autoBuilder,
        DriveSubsystem swerveDrive,
        ResetExtension resetExtension,
        PathPlannerTrajectory trajectory) {

            // resetExtension = new ResetExtension(extension);

        addCommands(
            resetExtension,
            new ParallelCommandGroup(
                extensionPID,
                rotationPID,
                new SequentialCommandGroup(
                    new InstantCommand(() -> {
                    extensionPID.setSetpoint(ExtensionSetpoint.HIGH);
                    rotationPID.setSetpoint(RotationSetpoint.HIGH);
                    intake.set(0.3);
                }, intake
                ),
                new WaitUntilCommand(rotationPID::atSetpoint),
                new WaitUntilCommand(extensionPID::atSetpoint),
                new RunCommand(
                    () -> {
                        intake.outTakeCone();
                    }, intake
                ).withTimeout(0.5),
                new InstantCommand(() -> {
                    extensionPID.setSetpoint(ExtensionSetpoint.RESET);
                    rotationPID.setSetpoint(RotationSetpoint.RESET);
                    intake.stop();
                }, intake
                ),
                new WaitUntilCommand(rotationPID::atSetpoint),
                new WaitUntilCommand(extensionPID::atSetpoint),
                autoBuilder.followPath(trajectory)
                )    
            )
        );


    }
}
