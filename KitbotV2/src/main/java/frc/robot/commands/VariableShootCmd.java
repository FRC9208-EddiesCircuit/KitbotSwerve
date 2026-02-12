package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DeflectorSubsystem;
import frc.robot.subsystems.IntakeShooterSubsystem;

public class VariableShootCmd extends Command{

    private IntakeShooterSubsystem intakeShooterSubsystem;
    private DeflectorSubsystem deflectorSubsystem;
    private Supplier<Double> shooterSpeedSupplier, deflectorSpeedSupplier;
    private double shooterSpeed, deflectorSpeed;

    public VariableShootCmd(IntakeShooterSubsystem intakeShooterSubsystem, DeflectorSubsystem deflectorSubsystem,
     Supplier<Double> shooterSpeedSupplier, Supplier<Double> deflectorSpeedSupplier){

        this.intakeShooterSubsystem = intakeShooterSubsystem;
        this.deflectorSubsystem = deflectorSubsystem;
        this.shooterSpeedSupplier = shooterSpeedSupplier;
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
        shooterSpeed = -0.665;//shooterSpeedSupplier.get();
        deflectorSpeed = 0.32;//deflectorSpeedSupplier.get();

        intakeShooterSubsystem.setIntakeShooterSpeed(shooterSpeed);
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
