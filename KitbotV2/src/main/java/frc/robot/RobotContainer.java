// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.commands.ClimbCmd;
import frc.robot.commands.IntakeCmd;
import frc.robot.commands.ShootCmd;
import frc.robot.commands.VariableIntakeCmd;
import frc.robot.commands.VariableShootCmd;
import frc.robot.commands.ReverseIntakeCmd;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.DeflectorSubsystem;
import frc.robot.subsystems.IntakeShooterSubsystem;
import com.pathplanner.lib.auto.AutoBuilder;

public class RobotContainer {
    private double MaxSpeed = 0.6 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = 0.7 * RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /*
     * TUNE TS
     */
    private PIDController rotationController = new PIDController(0.06,0,0);
    
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
    /*private SwerveDrivePoseEstimator mt2PoseEstimator = 
        new SwerveDrivePoseEstimator(
            drivetrain.getKinematics(),
            drivetrain.getPigeon2().getRotation2d(),//TODO
            new SwerveModulePosition[]{
                drivetrain.getModule(0).getCachedPosition(),    //FL
                drivetrain.getModule(1).getCachedPosition(),    //FR
                drivetrain.getModule(2).getCachedPosition(),    //BL
                drivetrain.getModule(3).getCachedPosition()     //BR
            },
            new Pose2d(),
            VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)), //Figure these out
            VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30))   //These too
        );*/
    private double rotationControlSignal;
    private double xRate;
    private double yRate;
    private double rotationalRate;
    
    private Rotation2d fieldRelativeAngleMt2;
    private Rotation2d yawInitialMt2;

    private Rotation2d yawSetpoint;
    private double yawCurrent;
    private double yawOffset;
    private Pose2d hubPose;
    private Optional<Alliance> allianceColor = DriverStation.getAlliance();
    //private boolean doRejectUpdate = false;

    private double forwardTargetingSpeed;
    private double forwardTargetingKP;

    private double rotTargetingSpeed;
    private double rotTargetingKP;

    private Pose2d redHubPose = new Pose2d(
        11.9154194,
        4.0346376,
        new Rotation2d(Math.PI/2)
    );
    private Pose2d blueHubPose = new Pose2d(
        4.6256194,
        4.0346376,
        new Rotation2d(Math.PI/2)
    );

    public RobotContainer() {
        configureBindings();
        hubPose = redHubPose;
        //setHubPose();
        rotationController.enableContinuousInput(-180, 180);
    }

    /*
     * FIND SOMETHING BETTER
     */
    private void setHubPose(){
        if (DriverStation.getAlliance().isPresent()) {
            if (allianceColor.get() == Alliance.Red) {
                hubPose = redHubPose;
            }else if(allianceColor.get() == Alliance.Blue) {
                hubPose = blueHubPose;
            }
        }else{
            hubPose = redHubPose;
        }
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driveJS.getRawAxis(1) * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driveJS.getRawAxis(0) * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(calcRotationalRate())//(-twistJS.getRawAxis(2) * MaxAngularRate
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        //controller.x().whileTrue(drivetrain.applyRequest(() -> brake));
        /*controller.y().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-controller.getLeftY(), -controller.getLeftX()))
        ));*/

        controller.axisGreaterThan(3, 0.5)
            .whileTrue(new IntakeCmd(intakeShooterSubsystem, deflectorSubsystem));
        controller.axisGreaterThan(2, 0.5)
            .whileTrue(new VariableIntakeCmd(intakeShooterSubsystem, deflectorSubsystem,
             () -> -(-driveJS.getRawAxis(2) + 1) / 2, () -> (-twistJS.getRawAxis(3) + 1) / 2));

        controller.x().whileTrue(new ShootCmd(intakeShooterSubsystem, deflectorSubsystem));
        controller.b().whileTrue(new ReverseIntakeCmd(intakeShooterSubsystem, deflectorSubsystem));
        controller.a().whileTrue(new VariableShootCmd(intakeShooterSubsystem, deflectorSubsystem,
            () -> -(-driveJS.getRawAxis(2) + 1) / 2, () -> (-twistJS.getRawAxis(3) + 1) / 2));
        controller.rightBumper().whileTrue(new ClimbCmd(climbSubsystem,
                                                            () -> -controller.getLeftY(),
                                                            () -> -controller.getRightY()));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        controller.back().and(controller.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        controller.back().and(controller.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        controller.start().and(controller.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        controller.start().and(controller.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        new JoystickButton(twistJS, 2).whileTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public void SYSOUT(){
        System.out.println("X: " + LimelightHelpers.getTargetPose3d_RobotSpace("limelight").getX());
        System.out.println("Y: " + LimelightHelpers.getTargetPose3d_RobotSpace("limelight").getY());
        System.out.println("Z: " + LimelightHelpers.getTargetPose3d_RobotSpace("limelight").getZ());
        System.out.println("R: " + (180/Math.PI) * LimelightHelpers.getTargetPose3d_RobotSpace("limelight").getRotation().getAngle());

    }

    public double calcXRate(){
        xRate = 0;

        return xRate;
    }
    public double calcYRate(){
        yRate = 0;

        return yRate;
    }

    public double calcRotationalRate(){
        rotationalRate = 0;
        if(twistJS.getRawButton(1)){
            rotationalRate = calcRotationControlSignal();   //pose to hub

        }else if(driveJS.getRawButton(1)){           
            rotationalRate = rotControl();                  //aim and range to tag
        }else{
            rotationalRate = -twistJS.getRawAxis(2) * MaxAngularRate;
        }
        return rotationalRate;
    }
    /*public double calcRotationalRate(){
        rotationalRate = 0;
        if(!twistJS.getRawButton(1)){
            rotationalRate = -twistJS.getRawAxis(2) * MaxAngularRate;
        }else{
            rotationalRate = calcRotationControlSignal();
        }
        return rotationalRate;
    }*/
    public double rangeControl(){
        forwardTargetingSpeed = 
            LimelightHelpers.getTY("limelight-anarchy") * forwardTargetingKP;
        forwardTargetingSpeed *= MaxSpeed;

        return forwardTargetingSpeed;
    }
    public double rotControl(){
        rotTargetingSpeed = 
            LimelightHelpers.getTX("limelight-anarchy") * rotTargetingKP;
        rotTargetingSpeed *= MaxAngularRate;
        return rotTargetingSpeed;
    }


    public double calcRotationControlSignal(){
        yawCurrent = drivetrain.getPoseEstimator().getEstimatedPosition().getRotation().getDegrees();//mt2PoseEstimator.getEstimatedPosition().getRotation().getDegrees();
        yawOffset = calcYawSetpoint().getDegrees();
        rotationControlSignal = rotationController.calculate(
            yawCurrent,
            yawCurrent + yawOffset
        );
        return rotationControlSignal;
    }

    public Rotation2d calcYawSetpoint(){

        yawInitialMt2 = drivetrain.getPoseEstimator().getEstimatedPosition().getRotation();//mt2PoseEstimator.getEstimatedPosition().getRotation();
        fieldRelativeAngleMt2 = new Rotation2d(
            hubPose.getX() - drivetrain.getPoseEstimator().getEstimatedPosition().getX(),//mt2PoseEstimator.getEstimatedPosition().getX(),
            hubPose.getY() - drivetrain.getPoseEstimator().getEstimatedPosition().getY()//mt2PoseEstimator.getEstimatedPosition().getY()
        );

        yawSetpoint = fieldRelativeAngleMt2.minus(yawInitialMt2);
        //System.out.println(mt2PoseEstimator.getEstimatedPosition().getTranslation().getDistance(hubPose.getTranslation()));
        return yawSetpoint;

    }
    public void updateOdometry(){
        drivetrain.updateOdometry();
    }

    /*public void updateOdometry(){
        //llmeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-anarchy");
        //drivetrain.addVisionMeasurement(llmeasurement.pose, llmeasurement.timestampSeconds, VecBuilder.fill(.7,.7,9999999));
        mt2PoseEstimator.update(
            drivetrain.getPigeon2().getRotation2d(),
            new SwerveModulePosition[]{
                drivetrain.getModule(0).getCachedPosition(),    //FL
                drivetrain.getModule(1).getCachedPosition(),    //FR
                drivetrain.getModule(2).getCachedPosition(),    //BL
                drivetrain.getModule(3).getCachedPosition()     //BR
            }
        );

        LimelightHelpers.SetRobotOrientation("limelight-anarchy", mt2PoseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-anarchy");
        if(Math.abs(drivetrain.getPigeon2().getAngularVelocityZDevice().getValueAsDouble()) > 720 || mt2.tagCount == 0) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
        {
            doRejectUpdate = true;
        }else{
            doRejectUpdate = false;
        }

        if(!doRejectUpdate)
        {

            mt2PoseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
            mt2PoseEstimator.addVisionMeasurement(
                mt2.pose,
                mt2.timestampSeconds);
        }
        
    }*/

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
