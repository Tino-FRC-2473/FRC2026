package frc.robot.systems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.generated.CommandSwerveDrivetrain;
import frc.robot.generated.TunerConstants;
import frc.robot.input.Input;
import frc.robot.input.InputTypes.ButtonInput;
import frc.robot.limelight.LimelightHelpers;
import frc.robot.input.InputTypes.AxialInput;

import static frc.robot.Constants.DrivetrainConstants.PATH_CONSTRAINTS;

public class Drivetrain extends FSMSystem<Drivetrain.DrivetrainState> {

    public enum DrivetrainState {
        TELEOP,
        PATHFIND,
        FACE_HUB,
        FACE_PASS,
        ENTRY
    }

    private static final LinearVelocity MAX_SPEED = TunerConstants.kSpeedAt12Volts;
    private static final AngularVelocity MAX_ANGULAR_SPEED = DrivetrainConstants.MAX_ANGULAR_VELOCITY;

    private final SwerveRequest.FieldCentric driveFieldCentric = new SwerveRequest.FieldCentric()
            .withDeadband(MAX_SPEED.in(MetersPerSecond) * DrivetrainConstants.TRANSLATIONAL_DEADBAND)
            .withRotationalDeadband(MAX_ANGULAR_SPEED.in(RadiansPerSecond) * DrivetrainConstants.ROTATIONAL_DEADBAND)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.ApplyRobotSpeeds applyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.FieldCentricFacingAngle driveFacingAngle = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(MAX_SPEED.in(MetersPerSecond) * DrivetrainConstants.TRANSLATIONAL_DEADBAND)
            .withRotationalDeadband(MAX_ANGULAR_SPEED.in(RadiansPerSecond) * DrivetrainConstants.ROTATIONAL_DEADBAND)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private DrivetrainState currentState;
    private CommandSwerveDrivetrain drivetrain;
    private Command pathfindCommand = null;
    private double invertControls = 1;
    private LinearFilter filter;

    private AprilTagFieldLayout field = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    private Field2d elasticfield = new Field2d();

    private double targetAngle = 0;

    public Drivetrain() {
        drivetrain = TunerConstants.createDrivetrain();
        filter = LinearFilter.singlePoleIIR(0.1, 0.02);
        SmartDashboard.putData(elasticfield);

        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
            throw new RuntimeException(e);
        }

        AutoBuilder.configure(
                this::getPose,
                drivetrain::resetPose,
                () -> drivetrain.getState().Speeds,
                (speeds, feedforwards) -> {
                    // Correcting the omega inversion often seen in older PP versions
                    ChassisSpeeds speedCorrected = new ChassisSpeeds(
                            speeds.vxMetersPerSecond,
                            speeds.vyMetersPerSecond,
                            -speeds.omegaRadiansPerSecond);

                    drivetrain.setControl(
                            applyRobotSpeeds
                                    .withSpeeds(speedCorrected.times(Constants.DrivetrainConstants.TRANSLATIONAL_DAMP))
                                    .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                                    .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons()));
                },
                new PPHolonomicDriveController(
                        new PIDConstants(ModuleConstants.DRIVE_P, ModuleConstants.DRIVE_I, ModuleConstants.DRIVE_D),
                        new PIDConstants(ModuleConstants.STEER_P, ModuleConstants.STEER_I, ModuleConstants.STEER_D)),
                config,
                () -> DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red,
                drivetrain);

        reset();
    }

    @Override
    public void reset() {
        currentState = DrivetrainState.ENTRY;
        stop();
        update(null);
    }

    @Override
    public void update(Input input) {
        drivetrain.periodic();
        // Remove CommandScheduler.run() if it's already in Robot.java periodic
        elasticfield.setRobotPose(drivetrain.getState().Pose);
        

        Logger.recordOutput("Drivetrain/Target Angle", targetAngle);
        Logger.recordOutput("Drivetrain/Target Angle Pose", getPose().rotateAround(getPose().getTranslation(), Rotation2d.fromRadians(targetAngle)));

        switch (currentState) {
            case TELEOP:
                handleTeleopState(input);
                break;
            case FACE_HUB:
                handleFaceTarget(input, getHubPose());
                break;
            case FACE_PASS:
                handleFaceTarget(input, getBestPassingTarget());
                break;
            case ENTRY:
            case PATHFIND:
                break;
            default:
                throw new IllegalStateException("[DRIVETRAIN] Invalid state: " + currentState);
        }

        currentState = nextState(input);
    }

    /* ======================== Helper Methods ======================== */

    private Pose2d getHubPose() {
        var alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
        return (alliance == DriverStation.Alliance.Red) ? DrivetrainConstants.RED_HUB_POSE
                : DrivetrainConstants.BLUE_HUB_POSE;
    }

    private Pose2d getBestPassingTarget() {
        var alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
        boolean isRed = (alliance == DriverStation.Alliance.Red);

        Pose2d outpost = isRed ? DrivetrainConstants.RED_OUTPOST_POSE : DrivetrainConstants.BLUE_OUTPOST_POSE;
        Pose2d pose3 = isRed ? DrivetrainConstants.RED_POSE3_POSE : DrivetrainConstants.BLUE_POSE3_POSE;

        double distOutpost = getPose().getTranslation().getDistance(outpost.getTranslation());
        double distPose3 = getPose().getTranslation().getDistance(pose3.getTranslation());

        return (distOutpost < distPose3) ? outpost : pose3;
    }

    private void handleFaceTarget(Input input, Pose2d targetPose) {
        if (input == null) return;

        double flipAlliance = (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue) ? 1.0 : -1.0;

        double xSpeed = -invertControls * flipAlliance * MathUtil.applyDeadband(
                input.getAxisValue(AxialInput.DRIVETRAIN_DRIVE_X), DrivetrainConstants.TRANSLATION_DEADBAND) * MAX_SPEED.in(MetersPerSecond);

        double ySpeed = -invertControls * flipAlliance * MathUtil.applyDeadband(
                input.getAxisValue(AxialInput.DRIVETRAIN_DRIVE_Y), DrivetrainConstants.TRANSLATION_DEADBAND) * MAX_SPEED.in(MetersPerSecond);

        // Transform2d distance = targetPose.minus(getPose());
        // Using angleModulus to prevent wrap-around issues with filtering
        Translation2d robotTranslation = getPose().getTranslation();
        Translation2d targetTranslation = targetPose.getTranslation();
        Translation2d diff = targetTranslation.minus(robotTranslation);
        targetAngle = Math.atan2(diff.getY(), diff.getX());
        // double currentAngle = getRotation();
        // double rawTarget = Math.atan2(distance.getY(), distance.getX());

        // double error = MathUtil.angleModulus(rawTarget - currentAngle);

        // if(Math.abs(error) > Math.PI / 2) {
        //     rawTarget = MathUtil.angleModulus(rawTarget + Math.PI);
        // }

        // targetAngle = MathUtil.angleModulus(rawTarget);
        
        //double currentAngle = drivetrain.getState().Pose.getRotation().getRadians();
    	//double angleError = Math.abs(MathUtil.angleModulus(targetAngle - currentAngle));
        //Logger.recordOutput("Drivetrain/Error", angleError);
        
        drivetrain.setControl(
                driveFacingAngle
                        .withTargetDirection(Rotation2d.fromRadians(targetAngle))
                        .withHeadingPID(2.5, 5, 0)
                        .withVelocityX(xSpeed * DrivetrainConstants.TRANSLATIONAL_DAMP)
                        .withVelocityY(ySpeed * DrivetrainConstants.TRANSLATIONAL_DAMP));
    }

    private void handleTeleopState(Input input) {
        if (input == null) return;

        double flipAlliance = (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue) ? 1.0 : -1.0;

        double xSpeed = -invertControls * flipAlliance * MathUtil.applyDeadband(
                input.getAxisValue(AxialInput.DRIVETRAIN_DRIVE_X), DrivetrainConstants.TRANSLATION_DEADBAND) * MAX_SPEED.in(MetersPerSecond);

        double ySpeed = -invertControls * flipAlliance * MathUtil.applyDeadband(
                input.getAxisValue(AxialInput.DRIVETRAIN_DRIVE_Y), DrivetrainConstants.TRANSLATION_DEADBAND) * MAX_SPEED.in(MetersPerSecond);

        double thetaSpeed = MathUtil.applyDeadband(
                input.getAxisValue(AxialInput.DRIVETRAIN_ROTATE), DrivetrainConstants.ROTATIONAL_DEADBAND) * MAX_ANGULAR_SPEED.in(RadiansPerSecond);

        drivetrain.setControl(
                driveFieldCentric
                        .withVelocityX(xSpeed * DrivetrainConstants.TRANSLATIONAL_DAMP)
                        .withVelocityY(ySpeed * DrivetrainConstants.TRANSLATIONAL_DAMP)
                        .withRotationalRate(thetaSpeed * DrivetrainConstants.ROTATIONAL_DAMP));

        if (input.getButtonPressed(ButtonInput.DRIVETRAIN_RESEED)) {
            drivetrain.seedFieldCentric();
        }
    }

    @Override
    protected DrivetrainState nextState(Input input) {
        if (input == null) return DrivetrainState.TELEOP;

        if (input.getButtonPressed(ButtonInput.INVERT_DRIVETRAIN_CONTROLS)) {
            invertControls *= -1;
        }

        switch (currentState) {
            case ENTRY:
                return hasDriverInput(input) ? DrivetrainState.TELEOP : DrivetrainState.ENTRY;

            case TELEOP:
                if (input.getButtonPressed(ButtonInput.DRIVETRAIN_PATHFIND)) {
                    setupAndStartPathfinding();
                    return DrivetrainState.PATHFIND;
                }
                if (input.getButtonValue(ButtonInput.FACE_HUB)) return DrivetrainState.FACE_HUB;
                if (input.getButtonValue(ButtonInput.FACE_PASS)) return DrivetrainState.FACE_PASS;
                return DrivetrainState.TELEOP;

            case PATHFIND:
                if (input.getButtonValue(ButtonInput.DRIVETRAIN_PATHFIND)) return DrivetrainState.PATHFIND;
                if (pathfindCommand != null) pathfindCommand.cancel();
                return DrivetrainState.TELEOP;

            case FACE_HUB:
                return input.getButtonValue(ButtonInput.FACE_HUB) /*&&  Math.abs(MathUtil.angleModulus(getRotation() - targetAngle) + Math.PI) >= Math.toRadians(20) */ ? DrivetrainState.FACE_HUB : DrivetrainState.TELEOP;

            case FACE_PASS:
                return input.getButtonValue(ButtonInput.FACE_PASS) &&  Math.abs(MathUtil.angleModulus(getRotation() - targetAngle) + Math.PI) >= Math.toRadians(20) ? DrivetrainState.FACE_PASS : DrivetrainState.TELEOP;

            default:
                return DrivetrainState.TELEOP;
        }
    }

    private void setupAndStartPathfinding() {
        var alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Red);
        int tagId = (alliance == DriverStation.Alliance.Red) ? 10 : 26;
        
        DrivetrainConstants.setTagToAlignTo(tagId);

        field.getTagPose(tagId).ifPresent(tagPose -> {
            Pose2d target = tagPose.toPose2d().transformBy(new Transform2d(
                -DrivetrainConstants.X_TRANFORM_FROM_TAG,
                DrivetrainConstants.Y_TRANFORM_FROM_TAG,
                Rotation2d.kZero
            ));
            startPathfinding(target);
        });
    }

    private void startPathfinding(Pose2d target) {
        Logger.recordOutput("Vision/AlignmentPose", target);
        pathfindCommand = AutoBuilder.pathfindToPose(target, PATH_CONSTRAINTS);
        pathfindCommand.schedule();
    }

    /* ======================== Boilerplate Getters ======================== */

    @AutoLogOutput(key = "Drivetrain/Pose")
    public Pose2d getPose() { return drivetrain.getState().Pose; }
    
    @AutoLogOutput(key = "Drivetrain/Rotation")
    public double getRotation() { 
        return MathUtil.angleModulus(drivetrain.getPigeon2().getRotation2d().getRadians() + Math.PI);
    }

	public Rotation3d getRotation3d() { return drivetrain.getPigeon2().getRotation3d(); }

    private boolean hasDriverInput(Input input) {
        return Math.abs(input.getAxisValue(AxialInput.DRIVETRAIN_DRIVE_X)) > 0.1
            || Math.abs(input.getAxisValue(AxialInput.DRIVETRAIN_DRIVE_Y)) > 0.1
            || Math.abs(input.getAxisValue(AxialInput.DRIVETRAIN_ROTATE)) > 0.1;
    }

    public void stop() {
        drivetrain.setControl(new SwerveRequest.Idle());
    }

	/**
	 * Adds a new timestamped vision measurement.
	 *
	 * @param visionPoseMeters The pose of the robot in the camera's coordinate
	 *                         frame
	 * @param timestampSeconds The timestamp of the measurement
	 * @param visionStdDevs    The standard deviations of the measurement in the x,
	 *                         y, and theta directions
	 */
	public void addVisionMeasurement(
			Pose2d visionPoseMeters,
			double timestampSeconds,
			Matrix<N3, N1> visionStdDevs) {
		drivetrain.addVisionMeasurement(new Pose2d(visionPoseMeters.getX(),
			visionPoseMeters.getY(),
			visionPoseMeters.getRotation().plus(Rotation2d.k180deg)),
			timestampSeconds, visionStdDevs);
		//drivetrain.addVisionMeasurement(visionPoseMeters, timestampSeconds,visionStdDevs);
	}
}