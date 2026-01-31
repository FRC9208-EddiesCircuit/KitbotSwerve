// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.commands.ClimbCmd;
import frc.robot.commands.IntakeCmd;
import frc.robot.commands.ShootCmd;
import frc.robot.commands.ReverseIntakeCmd;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.DeflectorSubsystem;
import frc.robot.subsystems.IntakeShooterSubsystem;

public class RobotContainer {
    private double MaxSpeed = 0.6 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = 0.7 * RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private PIDController rotationController = new PIDController(0.1,0,0);
    
    private IntakeShooterSubsystem intakeShooterSubsystem = new IntakeShooterSubsystem();
    private DeflectorSubsystem deflectorSubsystem = new DeflectorSubsystem();
    private ClimbSubsystem climbSubsystem = new ClimbSubsystem();

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController controller = new CommandXboxController(0);
    private final Joystick driveJS = new Joystick(2);
    private final Joystick twistJS = new Joystick(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private PoseEstimate llmeasurement;
    private double rotationControlSignal;
    private double rotationalRate;
    private Rotation2d fieldRelativeAngle;
    private Rotation2d yawInitial;
    private Rotation2d yawSetpoint;
    private Pose2d redHubPose = new Pose2d
    (
        11.9154194,
        4.0346376,
        new Rotation2d(Math.PI/2)
    );

    private Pose2d blueHubPose = new Pose2d
    (
        4.6256194,
        4.0346376,
        new Rotation2d(Math.PI/2)
    );

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(0)//driveJS.getRawAxis(1) * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(0)//driveJS.getRawAxis(0) * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(calcRotationalRate())
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        controller.x().whileTrue(drivetrain.applyRequest(() -> brake));
        controller.y().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-controller.getLeftY(), -controller.getLeftX()))
        ));

        controller.axisGreaterThan(2, 0.5)
            .whileTrue(new IntakeCmd(intakeShooterSubsystem, deflectorSubsystem));
        controller.a().whileTrue(new ShootCmd(intakeShooterSubsystem, deflectorSubsystem));
        controller.b().whileTrue(new ReverseIntakeCmd(intakeShooterSubsystem, deflectorSubsystem));
        controller.rightBumper().whileTrue(new ClimbCmd(climbSubsystem,
                                                            () -> controller.getLeftY(),
                                                            () -> controller.getRightY()));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        controller.back().and(controller.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        controller.back().and(controller.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        controller.start().and(controller.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        controller.start().and(controller.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        controller.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public double calcRotationalRate(){
        rotationalRate = 0;
        if(!twistJS.getRawButton(1)){
            rotationalRate = -twistJS.getRawAxis(2) * MaxAngularRate;
        }else{
            rotationalRate = calcRotationControlSignal();
        }
        return rotationalRate;
    }

    public double calcRotationControlSignal(){
        rotationControlSignal = rotationController.calculate(
            drivetrain.getState().Pose.getRotation().getDegrees(),//drivetrain.getPigeon2().getYaw().getValueAsDouble() % 360, drivetrain.getState().Pose.getRotation().getDegrees()
            calcYawSetpointRed().getDegrees()
        );
        return -rotationControlSignal;
    }

    public Rotation2d calcYawSetpointBlue(){
        llmeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
        if(llmeasurement != null && llmeasurement.tagCount > 0){
            yawInitial = llmeasurement.pose.getRotation();
            fieldRelativeAngle = new Rotation2d(
                blueHubPose.getX() - llmeasurement.pose.getX(), 
                blueHubPose.getY() - llmeasurement.pose.getY()
            );
        }else{
            yawInitial = Rotation2d.fromDegrees(0);
            fieldRelativeAngle = Rotation2d.fromDegrees(0);
        }
        yawSetpoint = fieldRelativeAngle.minus(yawInitial);
        return yawSetpoint;
    }

    public Rotation2d calcYawSetpointRed(){
        llmeasurement = LimelightHelpers.getBotPoseEstimate_wpiRed("limelight");
        if(llmeasurement != null && llmeasurement.tagCount > 0){
            yawInitial = llmeasurement.pose.getRotation();
            fieldRelativeAngle = new Rotation2d(
                redHubPose.getX() - llmeasurement.pose.getX(), 
                redHubPose.getY() - llmeasurement.pose.getY()
            );
        }else{
            yawInitial = Rotation2d.fromDegrees(0);
            fieldRelativeAngle = Rotation2d.fromDegrees(0);
        }
        //System.out.println("x: " + llmeasurement.pose.getX());
        //System.out.println("y: " + llmeasurement.pose.getY());

        yawSetpoint = fieldRelativeAngle.minus(yawInitial);
        return yawSetpoint;
        
    }
    public Command getAutonomousCommand() {
        // Simple drive forward auton
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
            // Reset our field centric heading to match the robot
            // facing away from our alliance station wall (0 deg).
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            // Then slowly drive forward (away from us) for 5 seconds.
            drivetrain.applyRequest(() ->
                drive.withVelocityX(0.5)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            )
            .withTimeout(5.0),
            // Finally idle for the rest of auton
            drivetrain.applyRequest(() -> idle)
        );
    }
}
