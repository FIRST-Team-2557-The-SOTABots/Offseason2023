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

public class TestAuto extends SequentialCommandGroup {
    

    public TestAuto (

    DriveSubsystem swerveDrive,
    ExtensionPID mExtensionPID,
    RotationPID mRotationPID,
    SwerveAutoBuilder autoBuilder,
    Intake mIntake,
    PathPlannerTrajectory trajectory,
    ResetExtension resetExtension

    ) {
        addCommands(
            resetExtension,
            new InstantCommand(() -> {
                autoBuilder.followPath(trajectory);
            }) 


        );

        


    }
}
