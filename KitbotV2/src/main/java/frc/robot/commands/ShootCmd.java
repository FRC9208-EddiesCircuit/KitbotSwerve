package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DeflectorSubsystem;
import frc.robot.subsystems.IntakeShooterSubsystem;

public class ShootCmd extends Command{

    private IntakeShooterSubsystem intakeShooterSubsystem;
    private DeflectorSubsystem deflectorSubsystem;
    private double minRPMs = 100;                                                //CHECK

    public ShootCmd(IntakeShooterSubsystem intakeShooterSubsystem, DeflectorSubsystem deflectorSubsystem){
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
        intakeShooterSubsystem.shoot();
        //if(intakeShooterSubsystem.getRPMs() > minRPMs){
            deflectorSubsystem.shootDeflection();
        //}
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
