// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.*;
import static frc.robot.Util.logf;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.utilities.SwerveModule;
import frc.robot.utilities.SwerveModuleIds;
import frc.robot.utilities.SwerveModuleType;
import static frc.robot.utilities.Util.round2;
public class DrivetrainSubsystem extends SubsystemBase {

  // The maximum voltage that will be delivered to the drive motors.
  public static final double MAX_VOLTAGE = 12.0;

  public static final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      // Front left
      new Translation2d(DRIVETRAIN_TRACK_WIDTH / 2.0, DRIVETRAIN_TRACK_LENGTH / 2.0),
      // Front right
      new Translation2d(DRIVETRAIN_TRACK_WIDTH / 2.0, -DRIVETRAIN_TRACK_LENGTH / 2.0),
      // Back left
      new Translation2d(-DRIVETRAIN_TRACK_WIDTH / 2.0, DRIVETRAIN_TRACK_LENGTH / 2.0),
      // Back right
      new Translation2d(-DRIVETRAIN_TRACK_WIDTH / 2.0, -DRIVETRAIN_TRACK_LENGTH / 2.0));

  // By default we use a Pigeon for our gyroscope. But if you use another
  // gyroscope, like a NavX, you can change this.
  // The important thing about how you configure your gyroscope is that rotating
  // the robot counter-clockwise should
  // cause the angle reading to increase until it wraps back over to zero.
  // FIXed Remove if you are using a Pigeon
  // private final PigeonIMU m_pigeon = new PigeonIMU(DRIVETRAIN_PIGEON_ID);
  // FIXed Uncomment if you are using a NavX
  public final AHRS m_navx = new AHRS(); // NavX connected over MXP

  // These are our modules. We initialize them in the constructor.
  private final SwerveModule m_frontLeftModule;
  private final SwerveModule m_frontRightModule;
  private final SwerveModule m_backLeftModule;
  private final SwerveModule m_backRightModule;

  private RobotContainer robotContainer;

  SwerveModule swerveModules[];

  private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

  public DrivetrainSubsystem(RobotContainer robotContainer) {
    this.robotContainer = robotContainer;

    m_frontLeftModule = new SwerveModule(0, SwerveModuleType.Mk4iL2, new SwerveModuleIds(
        FRONT_LEFT_MODULE_DRIVE_MOTOR, FRONT_LEFT_MODULE_STEER_MOTOR, FRONT_LEFT_MODULE_STEER_ENCODER));

    m_frontRightModule = new SwerveModule(1, SwerveModuleType.Mk4iL2, new SwerveModuleIds(
        FRONT_RIGHT_MODULE_DRIVE_MOTOR, FRONT_RIGHT_MODULE_STEER_MOTOR, FRONT_RIGHT_MODULE_STEER_ENCODER));

    m_backLeftModule = new SwerveModule(2, SwerveModuleType.Mk4nL2, new SwerveModuleIds(
        BACK_LEFT_MODULE_DRIVE_MOTOR, BACK_LEFT_MODULE_STEER_MOTOR, BACK_LEFT_MODULE_STEER_ENCODER));

    m_backRightModule = new SwerveModule(3, SwerveModuleType.Mk4nL2, new SwerveModuleIds(
        BACK_RIGHT_MODULE_DRIVE_MOTOR, BACK_RIGHT_MODULE_STEER_MOTOR, BACK_RIGHT_MODULE_STEER_ENCODER));

    swerveModules = new SwerveModule[] {
        m_frontLeftModule, m_frontRightModule, m_backLeftModule, m_backRightModule
    };
  }

  /**
   * Sets the gyroscope angle to zero. This can be used to set the direction the
   * robot is currently facing to the
   * 'forwards' direction.
   */
  public void zeroGyroscope() {
    // FIXed Uncomment if you are using a NavX
    // TODO: calibrate magneto navx
    // if (m_navx.isMagnetometerCalibrated()) {
    // // We will only get valid fused headings if the magnetometer is calibrated
    // // System.out.println("returning the angle FUSE ZERO from the robot:
    // // "+m_navx.getAngle());
    // // getFusedHeading() is an angle only valid if magnetic calibration has benn
    // // completed
    // zeroNavx = m_navx.getFusedHeading();
    // } else {
    // zeroNavx = -90;
    // }

    // Can do an angle adjustment using setAngleAdjustment it
    // Will adjust angle for getAngle does not impact getYaw
    // m_navx.setAngleAdjustment(90);
    // m_navx.reset();
    m_navx.zeroYaw();
    robotContainer.poseSubsystem.refreshGyroOffset();

    logf("zeroGyroscope fused:%.2f angle:%.2f\n",
        m_navx.getFusedHeading(), m_navx.getAngle());
  }

  // double zeroNavx = 0.0;

  public Rotation2d getGyroscopeRotation() {
    // if (m_navx.isMagnetometerCalibrated()) {
    // // // We will only get valid fused headings if the magnetometer is calibrated
    // // System.out.println("returning the angle FUSE ZERO from the robot:
    // // "+m_navx.getAngle());

    // Rotation2d r = Rotation2d.fromDegrees(-m_navx.getFusedHeading() + zeroNavx);
    // SmartDashboard.putNumber("Rot Cal", r.getDegrees());
    // return r;
    // }
    //
    // We have to invert the angle of the NavX so that rotating the robot
    // counter-clockwise makes the angle increase.
    // return Rotation2d.fromDegrees(360.0 - m_navx.getYaw());

    Rotation2d r = Rotation2d.fromDegrees(-m_navx.getYaw());
    SmartDashboard.putNumber("Rot NC", r.getDegrees());
    SmartDashboard.putNumber("Angle", m_navx.getAngle());
    return r;
  }

  public double getGyroscopeRotationRate() {
    return -m_navx.getRate();
  }

  public void drive(ChassisSpeeds chassisSpeeds) {
    m_chassisSpeeds = chassisSpeeds;
  }

  // Read the absolute values from the cancoder
  // CANCoder backLeft = new CANCoder(Constants.BACK_LEFT_MODULE_STEER_ENCODER);
  // double absPosDeg = backLeft.getAbsolutePosition();
  // double encPosDeg = backLeft.getPosition();
  @Override
  public void periodic() {
    if (Robot.count % 20 == -1) {
      for (SwerveModule mod : swerveModules) {
        SmartDashboard.putNumber(
            "Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
        SmartDashboard.putNumber(
            "Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
        SmartDashboard.putNumber(
            "Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
      }
    }

    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.MAX_VELOCITY_METERS_PER_SECOND);

    m_frontLeftModule.setDesiredState(states[0]);
    m_frontRightModule.setDesiredState(states[1]);
    m_backLeftModule.setDesiredState(states[2]);
    m_backRightModule.setDesiredState(states[3]);
    if (Robot.count % 20 == -1) { // disable Smartdash board display
      SmartDashboard.putNumber("FL", states[0].angle.getDegrees());
      SmartDashboard.putNumber("FR", states[1].angle.getDegrees());
      SmartDashboard.putNumber("BL", states[2].angle.getDegrees());
      SmartDashboard.putNumber("BR", states[3].angle.getDegrees());
      SmartDashboard.putNumber("Yaw", m_navx.getYaw());
      SmartDashboard.putNumber("Rot", getGyroscopeRotation().getDegrees());
    }
  }

  public void stop() {
    m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
  }

  /**
   * Gets the current drivetrain position, as reported by the modules themselves.
   * 
   * @return current drivetrain state. Array orders are frontLeft, frontRight,
   *         backLeft, backRight
   */
  public SwerveModulePosition[] getModulePositions() {
    Rotation2d flA = m_frontLeftModule.getCanCoder();
    Rotation2d frA = m_frontRightModule.getCanCoder();
    Rotation2d blA = m_backLeftModule.getCanCoder();
    Rotation2d brA = m_backRightModule.getCanCoder();
    double flP = m_frontLeftModule.getPosition();
    double frP = m_frontRightModule.getPosition();
    double blP = m_backLeftModule.getPosition();
    double brP = m_backRightModule.getPosition();
    if(Robot.count % 20 == -1){
      SmartDashboard.putNumber("flP", round2(flP));
      SmartDashboard.putNumber("frP", round2(frP));
      SmartDashboard.putNumber("blP", round2(blP));
      SmartDashboard.putNumber("brP", round2(brP));
    }

    // The postition is in meters
    return new SwerveModulePosition[] {
        new SwerveModulePosition(flP, flA),
        new SwerveModulePosition(frP, frA),
        new SwerveModulePosition(blP, blA),
        new SwerveModulePosition(brP, brA)
    };
  }

  public ChassisSpeeds getChassisSpeeds() {
    return m_chassisSpeeds;
  }
}
