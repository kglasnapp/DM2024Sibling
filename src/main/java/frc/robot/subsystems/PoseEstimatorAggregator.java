package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PoseEstimatorAggregator implements Supplier<Pose2d> {
    // mode -1 = aggregator uses the smar average between all the estimators
    // mode 0-n = returns the value of the estimator in the mode position. To use camera 1, 
    // set the mode to 0, camera 2 set it to 1 
    public int defaultMode = -1;
    public int mode = -1;
    public int count;
    public PoseEstimatorSubsystem poseEstimators[];
    private final Field2d field2d = new Field2d();

    public PoseEstimatorAggregator(PoseEstimatorSubsystem poseEstimators[]) {
        this.poseEstimators = poseEstimators;
        ShuffleboardTab tab = Shuffleboard.getTab("Vision Agreegator");
        tab.addString("Pose", this::getFomattedPose).withPosition(0, 0).withSize(2, 0);
        tab.add("Field", field2d).withPosition(2, 0).withSize(6, 4);
        SmartDashboard.putNumber("VisMod", defaultMode);        
    }

    private String getFomattedPose() {
        var pose = get();
        return String.format("(%.2f, %.2f) %.2f degrees",
                pose.getX(),
                pose.getY(),
                pose.getRotation().getDegrees());
    }

    @Override
    public Pose2d get() {
        count++;
        double x = 0;
        double y = 0;
        double angle = 0;
        double subsystemWeight[] = new double[poseEstimators.length];
        double total = 0;
        for (int i = 0; i < poseEstimators.length; ++i) {
            subsystemWeight[i] = poseEstimators[i].getAvg();
            total += subsystemWeight[i];
        }
        double a = 0;
        double b = 0;
        for (int i = 0; i < poseEstimators.length; ++i) {
            Pose2d pose = poseEstimators[i].getCurrentPose();
            x += pose.getX() * subsystemWeight[i];
            y += pose.getY() * subsystemWeight[i];
            a += Math.cos(pose.getRotation().getRadians())*subsystemWeight[i];
            b += Math.sin(pose.getRotation().getRadians())*subsystemWeight[i];
            // angle += Util.unNormalilzeAngle(pose.getRotation().getDegrees()) * subsystemWeight[i];
        }
        angle = Math.atan2(b,a);
        Pose2d pose = new Pose2d(x / total, y / total, new Rotation2d(angle));
        if (count % 20 == 0) {
            mode = (int) SmartDashboard.getNumber("VisMod",defaultMode);
            count = 0;
        }
        drawField(pose);
        if (mode < 0) {
            return pose;
        } else {
            return poseEstimators[mode].getCurrentPose();
        }
    }

    

    public void drawField(Pose2d pose) {
        field2d.setRobotPose(pose);
        // Conversion so robot appears where it actually is on field instead of always
        // on blue.
        if (DriverStation.getAlliance().get() == Alliance.Red) {
            field2d.setRobotPose(new Pose2d(FieldConstants.fieldLength - pose.getX(),
                    FieldConstants.fieldWidth - pose.getY(),
                    new Rotation2d(pose.getRotation().getRadians() + Math.PI)));
        } else {
            field2d.setRobotPose(pose);
        }
    }
}
