package frc.robot.commands.Autos;

import com.pathplanner.lib.path.PathPlannerTrajectory;

// import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
    AutoBuilder autoBuilder,
    Intake mIntake,
    PathPlannerTrajectory trajectory,
    ResetExtension resetExtension

    ) {
        addCommands(
            resetExtension,

            new ParallelCommandGroup(
                mExtensionPID,
                mRotationPID,

                new SequentialCommandGroup(
                    
                    new InstantCommand(() -> {
                        
                        mExtensionPID.setSetpoint(ExtensionSetpoint.HIGH);
                        mRotationPID.setSetpoint(RotationSetpoint.HIGH);

                    }),

                    new InstantCommand(() -> {
                        mIntake.set(0.3);
                    },mIntake
                    ),

                    new WaitUntilCommand(mExtensionPID::atSetpoint).withTimeout(5),
                    new WaitUntilCommand(mRotationPID::atSetpoint).withTimeout(5),

                    
                    new InstantCommand(() -> {

                        mIntake.set(-0.5);

                    }, mIntake
                    ).withTimeout(0.5),
                    
                    new WaitCommand(0.5),

                    new InstantCommand(() -> {

                        mExtensionPID.setSetpoint(ExtensionSetpoint.RESET);
                        mRotationPID.setSetpoint(RotationSetpoint.RESET);
                        
                        mIntake.stop();
                    }),

                    new WaitUntilCommand(mExtensionPID::atSetpoint).withTimeout(3),
                    new WaitUntilCommand(mRotationPID::atSetpoint).withTimeout(3)


                )

            )

            // new PrintCommand("testing")

        );

        


    }
}
