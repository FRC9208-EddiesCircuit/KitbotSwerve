package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class IntakeShooterSubsystem extends SubsystemBase {

    private TalonFX intakeShooterMotor= new TalonFX(50);
    private TalonFXConfiguration intakeShooterConfiguration = new TalonFXConfiguration();

    private double intakeSpeed = -0.3;                                                         //CHECK
    private double shooterSpeed = -0.7;                                                        //CHECK

    public IntakeShooterSubsystem(){
        intakeShooterConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; //CHECK
        intakeShooterConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;        //CHECK
        //intakeShooterConfiguration.withCurrentLimits(
            //new CurrentLimitsConfigs()
            //.withStatorCurrentLimit(120)                              //CHECK
            //.withStatorCurrentLimitEnable(true));   

        intakeShooterMotor.getConfigurator().apply(intakeShooterConfiguration);
    }

    public void intake(){
        intakeShooterMotor.set(intakeSpeed);
    }

    public void reverseIntake(){
        intakeShooterMotor.set(-intakeSpeed);
    }

    public void shoot(){
        intakeShooterMotor.set(shooterSpeed);
    }

    public void stop(){
        intakeShooterMotor.set(0);
    }

    public double getRPMs(){
        return intakeShooterMotor.getVelocity().getValueAsDouble();
    }
}
