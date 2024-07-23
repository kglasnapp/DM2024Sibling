package frc.robot.utilities;

import edu.wpi.first.wpilibj.RobotController;

public class Timeout {
    long startTime = 0;
    long endTime = 0;
    double timeOutSeconds = 0;

    public double timeSinceInitializedSeconds() {
        return (RobotController.getFPGATime() - startTime) / 1000000;
    }

    public double timeSinceInitializedMilli() {
        return (RobotController.getFPGATime() - startTime) / 1000;
    }

   public void setTimeout(double timeSeconds) {
        this.timeOutSeconds = timeSeconds;
        startTime = RobotController.getFPGATime();
        endTime = startTime + (long) (timeSeconds * 1000000);
    }

    public boolean isTimedOut() {
        return RobotController.getFPGATime() > endTime;
    }
}
