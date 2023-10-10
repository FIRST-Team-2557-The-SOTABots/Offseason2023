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


public class PlaceConeMobility extends SequentialCommandGroup {
    
    public static final double kTime = 1.5; //no idea what this is



    // Create new PlaceConeMobility
    public PlaceConeMobility (
        
        DriveSubsystem swerveDrive,
        ExtensionPID mExtensionPID,
        RotationPID mRotationPID,
        SwerveAutoBuilder autoBuilder,
        Intake mIntake,
        PathPlannerTrajectory trajectory,
        ResetExtension resetExtension) {

        addRequirements(swerveDrive);

        addCommands(
            resetExtension,
            // new InstantCommand(() -> {
            //     swerveDrive.resetOdometry(trajectory.getInitialState()); idk if this is even necessary
            // }) 
            new ParallelCommandGroup(
                mExtensionPID,
                mRotationPID,
                new SequentialCommandGroup(

                
                    // Extends and rotates arm to high node setpoints

                    new InstantCommand(() -> {
                        mRotationPID.setSetpoint(RotationSetpoint.HIGH);
                        mExtensionPID.setSetpoint(ExtensionSetpoint.HIGH);
                    }),


                    // Sets speed of intake

                    new InstantCommand(() -> {
                        mIntake.set(0.5);
                    }),


                    // Waits until rotation and extension pids are at setpoint

                    new WaitUntilCommand(mRotationPID::atSetpoint),
                    new WaitUntilCommand(mExtensionPID::atSetpoint),


                    // Outtake cone for 1 second, stop motors

                    new RunCommand(mIntake::outTakeCone, mIntake).withTimeout(1),
                    new InstantCommand(mIntake::stop),


                    // Resets arm extension and rotation to start config

                    new InstantCommand(() -> {
                        mRotationPID.setSetpoint(RotationSetpoint.RESET);
                        mExtensionPID.setSetpoint(ExtensionSetpoint.RESET);
                    }),


                    // Waits until extension and rotation pids are at setpoint

                    new WaitUntilCommand(mRotationPID::atSetpoint),
                    new WaitUntilCommand(mExtensionPID::atSetpoint),


                    // Follow Pathplanner path

                    autoBuilder.followPath(trajectory)
                )

            )
            

        
        );
        

    }


}
