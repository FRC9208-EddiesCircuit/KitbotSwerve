package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbCmd extends Command {

    private ClimbSubsystem climbSubsystem;

    private Supplier<Double> leftClimbInput;
    private Supplier<Double> rightClimbInput;

    private double leftClimbInputDouble;
    private double rightClimbInputDouble;
    
    public ClimbCmd(ClimbSubsystem climbSubsystem, Supplier<Double> leftClimbInput, Supplier<Double> rightClimbInput){
        this.climbSubsystem = climbSubsystem;
        this.leftClimbInput = leftClimbInput;
        this.rightClimbInput = rightClimbInput;
        addRequirements(climbSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        leftClimbInputDouble = leftClimbInput.get().doubleValue() * 0.5;
        rightClimbInputDouble = rightClimbInput.get().doubleValue() * 0.5;
        
        climbSubsystem.leftClimbMove(leftClimbInputDouble);
        climbSubsystem.rightClimbMove(rightClimbInputDouble);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        climbSubsystem.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
  }
}
