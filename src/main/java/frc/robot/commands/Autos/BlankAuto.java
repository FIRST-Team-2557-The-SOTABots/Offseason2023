package frc.robot.commands.Autos;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
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


public class BlankAuto extends SequentialCommandGroup{
    

    public BlankAuto(
    ExtensionPID extensionPID,
    RotationPID rotationPID,
    Intake intake,
    SwerveAutoBuilder autoBuilder,
    DriveSubsystem swerveDrive,
    ResetExtension resetExtension,
    PathPlannerTrajectory trajectory) {

        addCommands(

          new PrintCommand("This is a blank auto")  

        );



    }


}
