package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DeflectorSubsystem;
import frc.robot.subsystems.IntakeShooterSubsystem;

public class VariableIntakeCmd extends Command{

    private IntakeShooterSubsystem intakeShooterSubsystem;
    private DeflectorSubsystem deflectorSubsystem;
    private Supplier<Double> intakeSpeedSupplier, deflectorSpeedSupplier;
    private double intakeSpeed, deflectorSpeed;
    


    public VariableIntakeCmd(IntakeShooterSubsystem intakeShooterSubsystem, DeflectorSubsystem deflectorSubsystem, Supplier<Double> intakeSpeedSupplier, Supplier<Double> deflectorSpeedSupplier){
        this.intakeShooterSubsystem = intakeShooterSubsystem;
        this.deflectorSubsystem = deflectorSubsystem;
        this.intakeSpeedSupplier = intakeSpeedSupplier;
        this.deflectorSpeedSupplier = deflectorSpeedSupplier;
        addRequirements(intakeShooterSubsystem, deflectorSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        intakeSpeed = intakeSpeedSupplier.get();
        deflectorSpeed = -deflectorSpeedSupplier.get();

        intakeShooterSubsystem.setIntakeShooterSpeed(intakeSpeed);
        deflectorSubsystem.setDeflectionSpeed(deflectorSpeed);

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        intakeShooterSubsystem.stop();
        deflectorSubsystem.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
  }
}
