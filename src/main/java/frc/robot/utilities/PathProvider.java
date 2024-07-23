package frc.robot.utilities;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class PathProvider implements Supplier<TrapezoidProfile.State> {
    
    Pose2d destination;
    Pose2d currentPoint;

    Pose2d topLeft;
    Pose2d bottomRight;

    public TrapezoidProfile.State get() {


        return null;
    }

}
