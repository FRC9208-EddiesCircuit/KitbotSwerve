package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DeflectorSubsystem;
import frc.robot.subsystems.IntakeShooterSubsystem;

public class IntakeCmd extends Command{

    private IntakeShooterSubsystem intakeShooterSubsystem;
    private DeflectorSubsystem deflectorSubsystem;

    public IntakeCmd(IntakeShooterSubsystem intakeShooterSubsystem, DeflectorSubsystem deflectorSubsystem){
        this.intakeShooterSubsystem = intakeShooterSubsystem;
        this.deflectorSubsystem = deflectorSubsystem;
        addRequirements(intakeShooterSubsystem, deflectorSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        intakeShooterSubsystem.intake();
        deflectorSubsystem.intakeDeflection();

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
