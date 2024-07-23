package frc.robot.subsystems;

import static frc.robot.Util.logf;

import java.util.function.Supplier;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class LimeLightPoseSubsystem extends SubsystemBase implements Supplier<Pose2d> {
    private SwerveDrivePoseEstimator poseEstimator;
    NetworkTable table;
    NetworkTableEntry tx;
    NetworkTableEntry ty;
    NetworkTableEntry ta;
    NetworkTableEntry tv;
    NetworkTableEntry tl;
    NetworkTableEntry cl;
    DoubleArraySubscriber coordinatesSub;

    //NetworkTableEntry timestamp = table.getEntry("timestamp");
    ShuffleboardTab tab;
    private final Field2d field2d = new Field2d();
    Pose2d pose = new Pose2d(1.89, 0.5, new Rotation2d(Math.toRadians(180)));
    private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(10));
    private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(1, 1, Units.degreesToRadians(50));
    DrivetrainSubsystem drivetrainSubsystem;
    static int count = 0;
    public String cameraId;

    public LimeLightPoseSubsystem(DrivetrainSubsystem drivetrainSubsystem, String cameraId) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.cameraId = cameraId;
        this.table = NetworkTableInstance.getDefault().getTable(cameraId);
        coordinatesSub = table.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[0]);
        
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");
        tv = table.getEntry("tv");
        tab = Shuffleboard.getTab("Vision LimeLight "+(++count));
        tab.addString("Pose", this::getFomattedPose).withPosition(0, 0).withSize(2, 0);
        tab.add("Field", field2d).withPosition(2, 0).withSize(6, 4);
        poseEstimator = new SwerveDrivePoseEstimator(
                DrivetrainSubsystem.m_kinematics,
                drivetrainSubsystem.getGyroscopeRotation(),
                drivetrainSubsystem.getModulePositions(),
                new Pose2d(1.89, 0.5, new Rotation2d(Math.toRadians(180))),
                stateStdDevs,
                visionMeasurementStdDevs);                
    }

    double visionTimer = 0;
    public final static double VISION_POSE_FREQUENCY = 1000;

    double camearaToYawAdjustment = 0;
    Rotation2d yaw = new Rotation2d();

    public void setCurrentPose(Pose2d pose) {
        this.pose = pose;
        logf("Setting the new pose %s\n",pose.toString());
        poseEstimator = new SwerveDrivePoseEstimator(
                DrivetrainSubsystem.m_kinematics,
                drivetrainSubsystem.getGyroscopeRotation(),
                drivetrainSubsystem.getModulePositions(),
                pose,
                stateStdDevs,
                visionMeasurementStdDevs);
    }

    @Override
    public void periodic() {
        //read values periodically
        yaw = drivetrainSubsystem.getGyroscopeRotation();
        /**
         * https://www.chiefdelphi.com/t/limelight-odometry-question/433311
         * 
         * We get the TV value from the limelight and only add vision measurements if 
         * that limelight is currently seeing a target (TV = 1)
         * 
         */
        if (tv.getDouble(0.0) == 1.0) {
            // x and y are in degrees
            // double x = tx.getDouble(0.0);
            // double y = ty.getDouble(0.0);
            // double area = ta.getDouble(0.0);

            double coordinates[] = coordinatesSub.get();
            if (coordinates!=null && coordinates.length >= 2) {
                double x = coordinates[0];
                double y = coordinates[1];
                double area = ta.getDouble(0.0);
                if (Robot.count % 10 == 0) {
                    //post to smart dashboard periodically
                    SmartDashboard.putNumber("LimeLX", x);
                    SmartDashboard.putNumber("LimeLY", y);
                    SmartDashboard.putNumber("LimeLArea", area);
                }

                String pipeLine = "botpose_wpiblue"; // (Robot.alliance == Alliance.Red) ? "botpose_wpired" : "botpose_wpiblue";
                double llPose[] = NetworkTableInstance.getDefault().getTable(cameraId).getEntry(pipeLine)
                        .getDoubleArray(new double[7]);
                double cameraAngle = Math.toRadians(llPose[5]);
                camearaToYawAdjustment = cameraAngle - yaw.getRadians();
                Pose2d visionPose = new Pose2d(llPose[0], llPose[1], new Rotation2d(cameraAngle));
                //double timeS = RobotController.getFPGATime() / 1000000.0;

                /**
                 * https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization
                 * 
                 * pseudocode for the "latency" component of WPILib' addVisionMeasurement():
                 * 
                 * Timer.getFPGATimestamp() - (tl/1000.0) - (cl/1000.0) or Timer.getFPGATimestamp() - (botpose[6]/1000.0)
                 * 
                 */
                double timestamp = Timer.getFPGATimestamp() - llPose[6]/1000.0; 
                /**
                 * Other option to try:
                 * double timestamp = Timer.getFPGATimestamp() - (tl.getDouble(0.0)/1000.0) - (cl.getDouble(0.0)/1000.0)
                 */
                visionTimer = RobotController.getFPGATime() / 1000;
                poseEstimator.addVisionMeasurement(visionPose, timestamp);
            }
        }
        //camearaToYawAdjustment = 0;
        yaw = new Rotation2d(yaw.getRadians() - camearaToYawAdjustment);
        poseEstimator.update(yaw,
                drivetrainSubsystem.getModulePositions());
        pose = poseEstimator.getEstimatedPosition();
        if (Robot.count % 10 == 0) {
            field2d.setRobotPose(pose);
        }
    }

    public Pose2d getPose() {
        return pose;
    }

    private String getFomattedPose() {
        String s = String.format("(%.2f, %.2f) %.2f degrees",
                pose.getX(),
                pose.getY(),
                pose.getRotation().getDegrees());
        if (Robot.count % 250 == 0) {
            logf("Camera %s LL Pose %s yaw:%.2f\n", cameraId, s, yaw.getDegrees());
        }
        return s;
    }

    @Override
    public Pose2d get() {
        return pose;
    }
}