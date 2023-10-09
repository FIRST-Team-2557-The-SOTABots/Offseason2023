package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class JonasFunkyIntake extends CommandBase{
   private Intake mIntake;
   private DoubleSupplier mPower;
   
   public JonasFunkyIntake(Intake intake, DoubleSupplier power) {
    this.mIntake = intake;
    this.mPower = power;
    addRequirements(mIntake);
   }

   @Override
   public void execute() {
       double speed = mPower.getAsDouble() * 0.5;

       mIntake.set(speed);
   }
}
