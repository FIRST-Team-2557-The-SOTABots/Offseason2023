package frc.robot.commands.Autos;

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

public class IntakeCubeGround extends SequentialCommandGroup {
    
    public IntakeCubeGround(
        DriveSubsystem driveTrain,
        ExtensionPID extensionPID,
        RotationPID rotationPID,
        Intake intake,
        ResetExtension resetExtension
    ) {

        addCommands(
            
            new ParallelCommandGroup(
                
                extensionPID,
                rotationPID,

                new SequentialCommandGroup(
                    
                    new InstantCommand(() -> {
                        extensionPID.setSetpoint(ExtensionSetpoint.FLOOR);
                        rotationPID.setSetpoint(RotationSetpoint.FLOOR);
                    }),

                    new WaitUntilCommand(extensionPID::atSetpoint),
                    new WaitUntilCommand(rotationPID::atSetpoint),
                

                    new InstantCommand(() -> {

                        intake.set(-0.5);

                    }, intake
                    ).withTimeout(3),

                    new InstantCommand(() -> {

                        extensionPID.setSetpoint(ExtensionSetpoint.REST);
                        rotationPID.setSetpoint(RotationSetpoint.REST);

                    }).withTimeout(1)

                )

            )

        );


    }


}
