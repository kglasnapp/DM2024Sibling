package frc.robot.utilities;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PiecePickerPoseProvider implements Runnable {
    int count = 0;
    private DatagramSocket socket;
    private byte buffer[] = new byte[256];
    PieceEstimatedPose estimatedPose;
    RunningAverage areaRunningAverage = new RunningAverage(15);

    public PiecePickerPoseProvider() {
        try {
            socket = new DatagramSocket(5005);
            Thread t = new Thread(this);
            t.start();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public void run() {
        boolean running = true;
        try {
            while (running) {
                DatagramPacket packet = new DatagramPacket(buffer, buffer.length);
                socket.receive(packet);

                //double timestamp = ByteBuffer.wrap(buffer, 0, 8).order(ByteOrder.LITTLE_ENDIAN).getDouble();
                double x = ByteBuffer.wrap(buffer, 8, 8).order(ByteOrder.LITTLE_ENDIAN).getDouble();
                double y = ByteBuffer.wrap(buffer, 16, 8).order(ByteOrder.LITTLE_ENDIAN).getDouble();
                double angle = ByteBuffer.wrap(buffer, 24, 8).order(ByteOrder.LITTLE_ENDIAN).getDouble();
                double area = ByteBuffer.wrap(buffer, 32, 8).order(ByteOrder.LITTLE_ENDIAN).getDouble();
                double avg = areaRunningAverage.add(area);
                if (Math.abs(avg - area) > 10000) {
                    continue;
                }
                if (count % 10 == 0) {
                    SmartDashboard.putNumber("CCX", x);
                    SmartDashboard.putNumber("CCY", y);
                    SmartDashboard.putNumber("CCA", angle);
                    SmartDashboard.putNumber("CCAr", area);
                    count = 0;
                }
                Pose2d pose = new Pose2d(x, y, new Rotation2d(Math.toRadians(angle)));
                estimatedPose = new PieceEstimatedPose(pose, RobotController.getFPGATime() / 1000);
                count++;
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
        socket.close();
    }

    public boolean hasResult() {
        double millSecs = RobotController.getFPGATime() / 1000;
        return estimatedPose != null && millSecs - 500 <= estimatedPose.timestamp;
    }

    public PieceEstimatedPose getResult() {
        return estimatedPose;
    }

    public class PieceEstimatedPose {
        Pose2d pose;
        double timestamp;

        public PieceEstimatedPose(Pose2d pose, double timestamp) {
            this.pose = pose;
            this.timestamp = timestamp;
        }

        public Pose2d getPose() {
            return pose;
        }

        public double getTimestamp() {
            return timestamp;
        }
    }

}
