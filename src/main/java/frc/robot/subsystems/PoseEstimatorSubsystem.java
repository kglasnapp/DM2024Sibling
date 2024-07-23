package frc.robot.subsystems;

import java.io.IOException;
import java.util.ConcurrentModificationException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
// import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

//import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.math.geometry.Transform3d;

import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.utilities.CameraPoseEstimator;
//import frc.robot.Constants.DrivetrainConstants;
import frc.robot.utilities.RunningAverage;
import static frc.robot.utilities.Util.logf;

public class PoseEstimatorSubsystem extends SubsystemBase {

  private final PhotonCamera photonCamera;
  private final DrivetrainSubsystem drivetrainSubsystem;
  private final CameraPoseEstimator photonPoseEstimator;
  private final RunningAverage avg = new RunningAverage(120);

  // Kalman Filter Configuration. These can be "tuned-to-taste" based on how much
  // you trust your various sensors. Smaller numbers will cause the filter to
  // "trust" the estimate from that particular component more than the others.
  // This in turn means the particualr component will have a stronger influence
  // on the final pose estimate.

  /**
   * Standard deviations of model states. Increase these numbers to trust your
   * model's state estimates less. This
   * matrix is in the form [x, y, theta]ᵀ, with units in meters and radians, then
   * meters.
   */
  private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.2, 0.2, Units.degreesToRadians(5));
  // private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.025, 0.025,
  // Units.degreesToRadians(1));

  private Optional<EstimatedRobotPose> photonEstimatedRobotPose = Optional.empty();
  /**
   * Standard deviations of the vision measurements. Increase these numbers to
   * trust global measurements from vision
   * less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and
   * radians.
   */
  private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(10));
  // private static final Vector<N3> visionMeasurementStdDevs =
  // VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(10));
  // private static final Vector<N3> visionMeasurementStdDevs =
  // VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(10));

  private final SwerveDrivePoseEstimator poseEstimator;
  private final Field2d field2d = new Field2d();
  public Transform3d robotToCamera;
  String name;
  AprilTagFieldLayout layout;

  public PoseEstimatorSubsystem(String name, PhotonCamera photonCamera, Transform3d robotToCamera,
      DrivetrainSubsystem drivetrainSubsystem) {
    this.name = name;
    this.photonCamera = photonCamera;
    this.robotToCamera = robotToCamera;
    this.drivetrainSubsystem = drivetrainSubsystem;
    
    try {
      layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
      logf("In PoseEstimatorSubsystem the alliance is %s\n", DriverStation.getAlliance().get());
      layout.setOrigin(DriverStation.getAlliance().get() == Alliance.Blue ? OriginPosition.kBlueAllianceWallRightSide
          : OriginPosition.kRedAllianceWallRightSide);
    } catch (IOException e) {
      DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
      layout = null;
    }

    photonPoseEstimator = new CameraPoseEstimator(name, layout, PoseStrategy.LOWEST_AMBIGUITY, this.photonCamera,
        robotToCamera);

    poseEstimator = new SwerveDrivePoseEstimator(
        DrivetrainSubsystem.m_kinematics,
        drivetrainSubsystem.getGyroscopeRotation(),
        drivetrainSubsystem.getModulePositions(),
        new Pose2d(),
        stateStdDevs,
        visionMeasurementStdDevs);

    ShuffleboardTab tab = Shuffleboard.getTab("Vision " + name);
    tab.addString("Pose", this::getFomattedPose).withPosition(0, 0).withSize(2, 0);
    tab.add("Field", field2d).withPosition(2, 0).withSize(6, 4);
  }

  public double getAvg() {
    return avg.getAverage();
  }

  boolean lastValue = false;
  int lastValueCount = 0;
  int count = 0;
  @Override
  public void periodic() {

    if (count % 50 == 0) {
      layout.setOrigin(DriverStation.getAlliance().get() == Alliance.Blue ? OriginPosition.kBlueAllianceWallRightSide
          : OriginPosition.kRedAllianceWallRightSide);
    }

    count++;

    photonEstimatedRobotPose = photonPoseEstimator.update();
    if (photonEstimatedRobotPose.isPresent()) {
      EstimatedRobotPose pose = photonEstimatedRobotPose.get();

      // System.out.println("got a pose "+pose.estimatedPose);
      // Max distance you want a tag to be read at. Found issues after 15 feet away
      // from tag while moving.

      // if (Math.hypot(pose.estimatedPose.getX(), pose.estimatedPose.getY()) < 5.25)
      // {

      // Error with WPI code https://github.com/wpilibsuite/allwpilib/issues/4952
      try {
        // System.out.println("the distance is less than 15 feet");
        if (!lastValue) {
          // if (lastValueCount > 10) {
          SmartDashboard.putBoolean(name, true);
          lastValue = true;
          lastValueCount = 0;
          // }
        }
        lastValueCount++;
        try {
          poseEstimator.addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds);
          avg.add(3);
        } catch (Exception e) {
        }

      } catch (ConcurrentModificationException e) {
      }
      /** else statement for the hypotenuse */
      // } else {
      // avg.add(1);
      // if (lastValue) {
      // // if (lastValueCount > 10) {
      // SmartDashboard.putBoolean(name, false);
      // lastValue = false;
      // lastValueCount = 0;
      // // } else {
      // lastValueCount++;
      // // }
      // }
      // }
    } else {
      avg.add(1);
      if (lastValue) {
        // if (lastValueCount > 10) {
        SmartDashboard.putBoolean(name, false);
        lastValue = false;
        lastValueCount = 0;
        // } else {
        lastValueCount++;
        // }
      }
    }
    // Update pose estimator with drivetrain sensors
    poseEstimator.update(
        drivetrainSubsystem.getGyroscopeRotation(),
        drivetrainSubsystem.getModulePositions());

    field2d.setRobotPose(getCurrentPose());
    // Conversion so robot appears where it actually is on field instead of always
    // on blue.
    if (DriverStation.getAlliance().get() == Alliance.Red) {
      field2d.setRobotPose(new Pose2d(FieldConstants.fieldLength - getCurrentPose().getX(),
          FieldConstants.fieldWidth - getCurrentPose().getY(),
          new Rotation2d(getCurrentPose().getRotation().getRadians() + Math.PI)));
    } else {
      field2d.setRobotPose(getCurrentPose());
    }
  }

  private String getFomattedPose() {
    var pose = getCurrentPose();
    return String.format("(%.2f, %.2f) %.2f degrees",
        pose.getX(),
        pose.getY(),
        pose.getRotation().getDegrees());
  }

  public Pose2d getCurrentPose() {      
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the current pose to the specified pose. This should ONLY be called
   * when the robot's position on the field is known, like at the beginning of
   * a match.
   * 
   * @param newPose new pose
   */
  public void setCurrentPose(Pose2d newPose) {
    poseEstimator.resetPosition(
        drivetrainSubsystem.getGyroscopeRotation(),
        drivetrainSubsystem.getModulePositions(),
        newPose);
  }

  /**
   * Resets the position on the field to 0,0 0-degrees, with forward being
   * downfield. This resets
   * what "forward" is for field oriented driving.
   */
  public void resetFieldPosition() {
    setCurrentPose(new Pose2d());
  }


}