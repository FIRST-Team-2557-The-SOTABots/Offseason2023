package frc.robot.commands.Autos;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
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


public class CubeMobilBump extends SequentialCommandGroup{
    

    public CubeMobilBump(
    DriveSubsystem swerveDrive,
    ExtensionPID mExtensionPID,
    ResetExtension resetExtension,
    RotationPID mRotationPID,
    Intake mIntake
    
    // PathPlannerTrajectory trajectory
    ) {

        double kMobilityTimeout = 4.5;
        addCommands(
            resetExtension,
            new InstantCommand(() -> {
                swerveDrive.updatePose(swerveDrive.getPose());
            }),

            new ParallelCommandGroup(
                mExtensionPID,
                mRotationPID,

                new SequentialCommandGroup(
                    
                    new InstantCommand(() -> {
                        
                        mExtensionPID.setSetpoint(ExtensionSetpoint.MIDCONE);
                        mRotationPID.setSetpoint(RotationSetpoint.MIDCONE);

                    }),

                    new InstantCommand(() -> {
                        mIntake.set(-0.2);
                    },mIntake
                    ),

                    new WaitUntilCommand(mExtensionPID::atSetpoint).withTimeout(5),
                    new WaitUntilCommand(mRotationPID::atSetpoint).withTimeout(5),

                    
                    new InstantCommand(() -> {

                        mIntake.set(0.5);

                    }, mIntake
                    ).withTimeout(0.5),
                    
                    new WaitCommand(0.5),

                    new InstantCommand(() -> {

                        mExtensionPID.setSetpoint(ExtensionSetpoint.RESET);
                        mRotationPID.setSetpoint(RotationSetpoint.RESET);
                        
                        mIntake.stop();
                    }),

                    new WaitUntilCommand(mExtensionPID::atSetpoint).withTimeout(3),
                    new WaitUntilCommand(mRotationPID::atSetpoint).withTimeout(3),

                    new RunCommand(() ->
                        swerveDrive.drive(new ChassisSpeeds(-1,0,0)), swerveDrive
                    ).withTimeout(kMobilityTimeout),

                    new RunCommand(() -> {
                        swerveDrive.drive(0, 0, 0, true, false);; 
                    }
                    ).withTimeout(0.5)

            

                )
            )
        );



    }


}
