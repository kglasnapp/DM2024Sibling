package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import static frc.robot.Util.logf;

public class LimeLightPose extends SubsystemBase {

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-front");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    ShuffleboardTab tab;
    private final Field2d field2d = new Field2d();
    Pose2d pose = new Pose2d();

    public LimeLightPose() {
        tab = Shuffleboard.getTab("Vision Keith");
        tab.addString("Pose", this::getFomattedPose).withPosition(0, 0).withSize(2, 0);
        tab.add("Field", field2d).withPosition(2, 0).withSize(6, 4);
    }

    @Override
    public void periodic() {
        //read values periodically
        double x = tx.getDouble(0.0);
        double y = ty.getDouble(0.0);
        double area = ta.getDouble(0.0);
        double llPose[] = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose_wpiblue")
                .getDoubleArray(new double[6]);
        pose = new Pose2d(llPose[0], llPose[1], new Rotation2d(Math.toRadians(llPose[5])));
        field2d.setRobotPose(pose);
        //post to smart dashboard periodically
        SmartDashboard.putNumber("LimeLX", x);
        SmartDashboard.putNumber("LimeLY", y);
        SmartDashboard.putNumber("LimeLArea", area);
    }

    private String getFomattedPose() {
        String s = String.format("(%.2f, %.2f) %.2f degrees",
                pose.getX(),
                pose.getY(),
                pose.getRotation().getDegrees());
        if (Robot.count % 500 == 0) {
           // logf("Pose %s\n", s);
        }
        return s;
    }
}