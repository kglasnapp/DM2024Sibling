
package frc.robot.utilities;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

/**
 * Monitors the health of the robot loop by calculating the loops per second and
 * watching for garbage collection pauses.
 */
public class LoopMonitor {
    private int loopsThisSecond = 0;
    // Timestamps in microseconds
    private long lastLoop = 0;
    private long lastSecond = 0;
    private long shortestGap = 1000000;
    private long longestGap = 0;
    public int largeGapFaults = 0;
    private final int TIMES = 50;
    private boolean logTimes = false;
    long[] segmentTime = new long[TIMES];

    public LoopMonitor() {
    }

    /**
     * Called every iteration of the robot loop.
     */
    public void loop(long loopStart) {
        segmentTime[(int) Robot.count % TIMES] = RobotController.getFPGATime() - loopStart;
        final long elapsed = (loopStart - lastLoop) / 1000;
        if (elapsed > 35) {
            Util.logf("Time between periodic lasted %d ms! for count:%d\n", elapsed, Robot.count);
            largeGapFaults++;
        }
        if (elapsed > longestGap)
            longestGap = elapsed;
        if (elapsed < shortestGap)
            shortestGap = elapsed;
        loopsThisSecond++;
        if ((loopStart - lastSecond) >= 1000000) {
            SmartDashboard.putNumber("Loops Per Second", loopsThisSecond);
            SmartDashboard.putNumber("Smallest Loop:", shortestGap);
            SmartDashboard.putNumber("Largest Loop", longestGap);
            SmartDashboard.putNumber("Num Large Loops", largeGapFaults);
            loopsThisSecond = 0;
            longestGap = 0;
            shortestGap = 10000000;
            lastSecond = loopStart;
        }
        lastLoop = loopStart;
        if (Robot.count % TIMES == 0 && logTimes)
            logTimes();
    }

    void logTimes() {
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < TIMES; i++) {
            sb.append(segmentTime[i]);
            sb.append(',');
        }
        Util.logf("Times %d %s\n", Robot.count - TIMES, sb.toString());
    }

}
