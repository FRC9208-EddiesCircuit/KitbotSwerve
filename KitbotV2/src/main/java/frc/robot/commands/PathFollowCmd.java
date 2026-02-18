package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.LimelightHelpers;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;


public class PathFollowCmd extends Command{
    
    public CommandSwerveDrivetrain swerveDrivetrain;
    public SwerveDrivePoseEstimator swerveDrivePoseEstimator;

    private double MaxSpeed = 0.75 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    public final Pose2d goalPose;
    public PathPlannerTrajectoryState goalState;
    public Pose2d diff;

    private PPHolonomicDriveController mDriveController = new PPHolonomicDriveController(
        new PIDConstants(0, 0, 0),
        new PIDConstants(0, 0, 0)
    );

    public static final Rotation2d kRotationTolerance = Rotation2d.fromDegrees(2.0);
    public static final Distance kPositionTolerance = Inches.of(0.4);
    public static final LinearVelocity kSpeedTolerance = InchesPerSecond.of(0.25);
    public static final Time kEndTriggerDebounce = Seconds.of(0.04);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.Velocity);

    private final Trigger endTrigger;
    private final Trigger endTriggerDebounced;

    private double xControl;
    private double yControl;
    private double rControl;


    private final BooleanPublisher endTriggerLogger = NetworkTableInstance.getDefault().getTable("logging").getBooleanTopic("PositionPIDEndTrigger").publish();

    private PathFollowCmd(CommandSwerveDrivetrain swerveDrivetrain, Pose2d goalPose, SwerveDrivePoseEstimator swerveDrivePoseEstimator) {
        this.swerveDrivetrain = swerveDrivetrain;
        this.goalPose = goalPose;

        
        endTrigger = new Trigger(() -> aligned());
        endTriggerDebounced = endTrigger.debounce(kEndTriggerDebounce.in(Seconds));
    }

    private boolean aligned(){
        diff = swerveDrivePoseEstimator.getEstimatedPosition().relativeTo(goalPose);

        boolean rotation = MathUtil.isNear(
            0.0, 
            diff.getRotation().getRotations(), 
            kRotationTolerance.getRotations(), 
            0.0, 
            1.0
        );
        boolean position = diff.getTranslation().getNorm() < kPositionTolerance.in(Meters);
        boolean speed = Math.hypot(swerveDrivetrain.getState().Speeds.vxMetersPerSecond, swerveDrivetrain.getState().Speeds.vyMetersPerSecond) < kSpeedTolerance.in(MetersPerSecond);
        
        return rotation && position && speed;
    } 

    @Override
    public void initialize() {
        endTriggerLogger.accept(endTrigger.getAsBoolean());
    }

    @Override
    public void execute() {
        goalState = new PathPlannerTrajectoryState();
        goalState.pose = goalPose;

        endTriggerLogger.accept(endTrigger.getAsBoolean());

        xControl = mDriveController.calculateRobotRelativeSpeeds(swerveDrivePoseEstimator.getEstimatedPosition(), goalState).vxMetersPerSecond;
        yControl = mDriveController.calculateRobotRelativeSpeeds(swerveDrivePoseEstimator.getEstimatedPosition(), goalState).vyMetersPerSecond;
        rControl = mDriveController.calculateRobotRelativeSpeeds(swerveDrivePoseEstimator.getEstimatedPosition(), goalState).omegaRadiansPerSecond;

        swerveDrivetrain.applyRequest(() ->
            drive
                .withVelocityX(xControl)
                .withVelocityX(yControl)
                .withRotationalRate(rControl)
        );
    }

    @Override
    public void end(boolean interrupted) {
        endTriggerLogger.accept(endTrigger.getAsBoolean());
    }

    @Override
    public boolean isFinished() {
        return endTriggerDebounced.getAsBoolean();
    }
}