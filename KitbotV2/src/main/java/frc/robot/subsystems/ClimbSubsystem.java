package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {
    
    private TalonFX leftClimbMotor = new TalonFX(10000);
    private TalonFXConfiguration leftClimbConfiguration = new TalonFXConfiguration();

    private TalonFX rightClimbMotor = new TalonFX(10001);
    private TalonFXConfiguration rightClimbConfiguration = new TalonFXConfiguration();

    public ClimbSubsystem() {
        leftClimbConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        rightClimbConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    }

    public void leftClimbMove(double rotationSpeed) {
        leftClimbMotor.set(rotationSpeed);
    }

    public void rightClimbMove(double rotationSpeed) {
        rightClimbMotor.set(rotationSpeed);
    }

    public void stop() {
        leftClimbMotor.stopMotor();
        rightClimbMotor.stopMotor();
    }
}
