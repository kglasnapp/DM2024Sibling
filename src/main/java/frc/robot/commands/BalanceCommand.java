package frc.robot.commands;


import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.utilities.SwerveModule;

import static frc.robot.utilities.Util.logf;

public class BalanceCommand extends Command {
    public final static double PITCH_THRESHOLD = 2;
    public final static double ROLL_THRESHOLD = 2;
    public final static double MAX_VELOCITY = 0.004;
    private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(0.8, 0.2);
    private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(0.8, 0.2);

    private final ProfiledPIDController xController = new ProfiledPIDController(0.005, 0, 0, X_CONSTRAINTS);
    private final ProfiledPIDController yController = new ProfiledPIDController(0.005, 0, 0, Y_CONSTRAINTS);

    double initTime = 0;

    public static enum State {
        FAST_BACK, SLOW_BACK, CHECK_IF_NEED_RETRY, RETRY, BALANCING, 
        LOCK_WHEELS, LOCK_WHEELS_COMPLETE, CHECK_BALANCE, FINISHED
    }

    static State state = State.FAST_BACK;

    DrivetrainSubsystem drivetrainSubsystem;

    double zeroPitch = 0;
    double zeroRoll = 0;

    public BalanceCommand(DrivetrainSubsystem drivetrainSubsystem) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        //SwerveModule.powerRatio = SwerveModule.NORMAL;
        addRequirements(drivetrainSubsystem);
        xController.setTolerance(2);
        yController.setTolerance(2);
    }

    public void zeroGyroscope() {        
        zeroRoll = drivetrainSubsystem.m_navx.getRoll();
        zeroPitch = drivetrainSubsystem.m_navx.getPitch();
        xController.reset(drivetrainSubsystem.m_navx.getRoll());
        yController.reset(drivetrainSubsystem.m_navx.getPitch());
    }

    @Override
    public void initialize() {
        zeroGyroscope();
        initTime = RobotController.getFPGATime() / 1000;
        state = State.FAST_BACK;      
        DefaultDriveCommand.autonomous = true;
        SwerveModule.setPowerRatio(1.4); 
    }

    @Override
    public void execute() {
        double pitch = drivetrainSubsystem.m_navx.getPitch() - zeroPitch;
        double roll = drivetrainSubsystem.m_navx.getRoll() - zeroRoll;
        double yaw = drivetrainSubsystem.m_navx.getYaw();
        logf("State = %s yaw = %.2f pitch = %.2f roll = %.2f\n", state, yaw, pitch, roll);
        if (state == State.FAST_BACK) {
            drivetrainSubsystem.drive(
                    new ChassisSpeeds(
                            0.02,
                            0,
                            0));
            if (RobotController.getFPGATime() / 1000 - initTime > 2000) {
                state = State.SLOW_BACK;
                drivetrainSubsystem.drive(
                            new ChassisSpeeds(
                                    0.02,
                                    0,
                                    0));
                initTime = RobotController.getFPGATime() / 1000;
            }
        }
        if (state == State.LOCK_WHEELS) {
            initTime = RobotController.getFPGATime() / 1000;
            drivetrainSubsystem.drive(
                            new ChassisSpeeds(
                                    0,
                                    0.01,
                                    0));
            state = State.LOCK_WHEELS_COMPLETE;
            return;                                    
        }
        if (state == State.LOCK_WHEELS_COMPLETE) {
            if (RobotController.getFPGATime() / 1000 - initTime > 100) {                
                state = State.FINISHED;
                initTime = RobotController.getFPGATime() / 1000;
                drivetrainSubsystem.stop();
            }
        }
        if (state == State.CHECK_BALANCE) {
            if (RobotController.getFPGATime() / 1000 - initTime > 800) {                
                if (Math.abs(roll)<3) {
                    state = State.FINISHED;
                } else {
                    initTime = (RobotController.getFPGATime() / 1000) - 800;
                    double speed = 0.015;
                    if (roll < 0) {
                        speed = -0.015;
                    }
                    drivetrainSubsystem.drive(
                            new ChassisSpeeds(
                                    speed,
                                    0,
                                    0));
                    state = State.SLOW_BACK;                                    
                }
            }
        }
        if (state == State.SLOW_BACK) {
            if (Math.abs(roll)< 8 || RobotController.getFPGATime() / 1000 - initTime > 1100) {
                state = State.LOCK_WHEELS;
                drivetrainSubsystem.drive(
                            new ChassisSpeeds(
                                    0,
                                    0,
                                    0));
                initTime = RobotController.getFPGATime() / 1000;
            }
        }
        if (state == State.CHECK_IF_NEED_RETRY) {
            if (RobotController.getFPGATime() / 1000 - initTime > 400) {
                if (Math.abs(pitch) > 3 || Math.abs(roll) > 3) {
                    state = State.BALANCING;
                    drivetrainSubsystem.drive(
                            new ChassisSpeeds(
                                    0,
                                    0,
                                    0));
                    logf("Changing state to Balancing, pitch = %.2f and roll = %.2f\n", pitch, roll);
                } else {                    
                    state = State.RETRY;
                    drivetrainSubsystem.drive(
                        new ChassisSpeeds(
                                -5,
                                0,
                                0));
                    initTime = RobotController.getFPGATime() / 1000;
                }
            }
        }
        if (state == State.RETRY) {
            if (RobotController.getFPGATime() / 1000 - initTime > 1000) {
                state = State.FAST_BACK;
                initTime = RobotController.getFPGATime() / 1000;
            }
        }
        if (state == State.BALANCING) {

            if (Robot.count % 20 == 0) {
                SmartDashboard.putNumber("Pitch", pitch);
                SmartDashboard.putNumber("Roll", roll);
            }

            xController.setGoal(0);
            yController.setGoal(0);

            if (Math.abs(roll) < ROLL_THRESHOLD &&
                    Math.abs(pitch) < PITCH_THRESHOLD) {
                drivetrainSubsystem.stop();
                return;
            }

            double xSpeed = xController.calculate(roll); 
            double ySpeed = yController.calculate(pitch);
            double omegaSpeed = 0;

            if (Math.abs(pitch) < ROLL_THRESHOLD) {
                ySpeed = 0;
            }
            if (Math.abs(roll) < PITCH_THRESHOLD) {
                xSpeed = 0;
            }
            if (Robot.count % 20 == 0) {
                SmartDashboard.putNumber("xSpeed", xSpeed);
                SmartDashboard.putNumber("ySpeed", ySpeed);
            }

            drivetrainSubsystem.drive(
                    new ChassisSpeeds(xSpeed, ySpeed, omegaSpeed));
        }
    }

    double getSpeed(double angleInDegrees) {
        return angleInDegrees * MAX_VELOCITY;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.stop();
        DefaultDriveCommand.autonomous = false;
        SwerveModule.setPowerRatio(SwerveModule.TURBO); 
    }

    @Override
    public boolean isFinished() {
        double pitch = drivetrainSubsystem.m_navx.getPitch() - zeroPitch;
        double roll = drivetrainSubsystem.m_navx.getRoll() - zeroRoll;
        return state == State.FINISHED || (state == State.BALANCING && Math.abs(pitch) < PITCH_THRESHOLD && Math.abs(roll) < ROLL_THRESHOLD);
    }
}