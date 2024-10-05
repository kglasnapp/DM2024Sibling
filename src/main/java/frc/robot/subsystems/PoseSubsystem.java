package frc.robot.subsystems;

import static frc.robot.Util.logf;

import java.util.function.Supplier;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.utilities.LimelightHelpers;

public class PoseSubsystem extends SubsystemBase implements Supplier<Pose2d> {
    // private static final Pose2d STARTING_POSE = new Pose2d(1.89, 0.5, new
    // Rotation2d(Math.toRadians(180)));
    private static final Pose2d STARTING_POSE_RED = new Pose2d(new Translation2d(), Rotation2d.fromDegrees(180));
    private static final Pose2d STARTING_POSE_BLUE = new Pose2d();
    private static final boolean USE_VISION = true;

    // Defines the accuracy of the different position sources
    // Numbers are standard deviations in x, y, rot order
    private static final Vector<N3> ODOMETRY_ACCURACY = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(1));
    private static final Vector<N3> VISION_ACCURACY_MT1 = VecBuilder.fill(2.0, 2.0, Units.degreesToRadians(20));
    private static final Vector<N3> VISION_ACCURACY_MT2 = VecBuilder.fill(1.0, 1.0, 9999999);

    private DrivetrainSubsystem drivetrainSubsystem;
    private SwerveDrivePoseEstimator poseEstimator;
    private String cameraId;

    private final Field2d field2d = new Field2d();
    private ShuffleboardTab tab;

    // TODO: Consider having this default to true
    private boolean assumeNextVisionPose = false;
    private boolean useMegaTag2 = false;

    public PoseSubsystem(DrivetrainSubsystem drivetrainSubsystem, String cameraId) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.cameraId = cameraId;

        tab = Shuffleboard.getTab("Odometry");
        tab.addString("Pose", this::getFormattedPose).withPosition(0, 0).withSize(2, 0);
        tab.add("Field", field2d).withPosition(2, 0).withSize(6, 4);

        if (DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
            poseEstimator = new SwerveDrivePoseEstimator(
                    DrivetrainSubsystem.m_kinematics,
                    drivetrainSubsystem.getGyroscopeRotation(),
                    drivetrainSubsystem.getModulePositions(),
                    STARTING_POSE_RED,
                    ODOMETRY_ACCURACY,
                    VISION_ACCURACY_MT1);
        } else {
            poseEstimator = new SwerveDrivePoseEstimator(
                    DrivetrainSubsystem.m_kinematics,
                    drivetrainSubsystem.getGyroscopeRotation(),
                    drivetrainSubsystem.getModulePositions(),
                    STARTING_POSE_BLUE,
                    ODOMETRY_ACCURACY,
                    VISION_ACCURACY_MT1);
        }
    }

    public void setCurrentPose(Pose2d pose) {
        logf("Setting the new pose %s angle:%.2f yaw:%.2f\n", pose.toString(),
                drivetrainSubsystem.m_navx.getAngle(), drivetrainSubsystem.m_navx.getYaw());

        poseEstimator.resetPosition(drivetrainSubsystem.getGyroscopeRotation(),
                drivetrainSubsystem.getModulePositions(), pose);

        // useMegaTag2 = true;
    }

    public void refreshGyroOffset() {
        poseEstimator.resetPosition(drivetrainSubsystem.getGyroscopeRotation(),
                drivetrainSubsystem.getModulePositions(), poseEstimator.getEstimatedPosition());
    }

    public void assumeNextVisionPose() {
        assumeNextVisionPose = true;
    }

    @Override
    public void periodic() {
        // Update Pose Estimate with odometry
        poseEstimator.update(
                drivetrainSubsystem.getGyroscopeRotation(),
                drivetrainSubsystem.getModulePositions());

        boolean doRejectUpdate = false;

        if (USE_VISION) {
            if (useMegaTag2 && !assumeNextVisionPose) {
                LimelightHelpers.SetRobotOrientation(cameraId,
                        poseEstimator.getEstimatedPosition().getRotation().getDegrees(),
                        drivetrainSubsystem.getGyroscopeRotationRate(), 0, 0, 0, 0);
                LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraId);

                if (mt2 != null) {
                    // if our angular velocity is greater than 720 degrees per second,
                    // ignore vision updates
                    if (Math.abs(drivetrainSubsystem.getGyroscopeRotationRate()) > 720) {
                        doRejectUpdate = true;
                    }
                    if (mt2.tagCount == 0) {
                        doRejectUpdate = true;
                    }
                    if (!doRejectUpdate) {
                        if (!assumeNextVisionPose) {
                            poseEstimator.setVisionMeasurementStdDevs(VISION_ACCURACY_MT2);
                            poseEstimator.addVisionMeasurement(
                                    mt2.pose,
                                    mt2.timestampSeconds);
                        } else {
                            setCurrentPose(mt2.pose);
                            assumeNextVisionPose = false;
                        }
                    }
                }
            } else {
                LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(cameraId);

                if (mt1 != null) {
                    // More rigorous checks when only one april tag is seen
                    if (mt1.tagCount == 1 && mt1.rawFiducials.length == 1) {
                        if (mt1.rawFiducials[0].ambiguity > .6) {
                            doRejectUpdate = true;
                        }
                        if (mt1.rawFiducials[0].distToCamera > 3) {
                            doRejectUpdate = true;
                        }
                    }

                    // Ignore spurious updates with no april tags visible
                    if (mt1.tagCount == 0) {
                        doRejectUpdate = true;
                    }

                    if (!doRejectUpdate) {
                        if (!assumeNextVisionPose) {
                            poseEstimator.setVisionMeasurementStdDevs(VISION_ACCURACY_MT1);
                            poseEstimator.addVisionMeasurement(
                                    mt1.pose,
                                    mt1.timestampSeconds);
                        } else {
                            setCurrentPose(mt1.pose);
                            assumeNextVisionPose = false;
                        }
                    }
                }
            }
        }

        if (Robot.count % 10 == 0) {
            field2d.setRobotPose(get());
        }
        if (Robot.count % 250 == 0) {
            logf("Pose %s angle:%.2f yaw:%.2f\n", getFormattedPose(),
                    drivetrainSubsystem.m_navx.getAngle(), drivetrainSubsystem.m_navx.getYaw());
        }
    }

    private String getFormattedPose() {
        Pose2d pose = get();

        String s = String.format("(%.2f, %.2f) %.2f degrees",
                pose.getX(),
                pose.getY(),
                pose.getRotation().getDegrees());

        return s;
    }

    @Override
    public Pose2d get() {
        return poseEstimator.getEstimatedPosition();
    }

    public boolean hasGoodPose() {
        return USE_VISION;
    }
}