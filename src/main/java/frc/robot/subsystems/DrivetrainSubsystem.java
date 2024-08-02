// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.BACK_LEFT_MODULE_DRIVE_MOTOR;
import static frc.robot.Constants.BACK_LEFT_MODULE_STEER_ENCODER;
import static frc.robot.Constants.BACK_LEFT_MODULE_STEER_MOTOR;
import static frc.robot.Constants.BACK_LEFT_MODULE_STEER_OFFSET;
import static frc.robot.Constants.BACK_RIGHT_MODULE_DRIVE_MOTOR;
import static frc.robot.Constants.BACK_RIGHT_MODULE_STEER_ENCODER;
import static frc.robot.Constants.BACK_RIGHT_MODULE_STEER_MOTOR;
import static frc.robot.Constants.BACK_RIGHT_MODULE_STEER_OFFSET;
import static frc.robot.Constants.DRIVETRAIN_TRACKWIDTH_METERS;
import static frc.robot.Constants.DRIVETRAIN_WHEELBASE_METERS;
import static frc.robot.Constants.FRONT_LEFT_MODULE_DRIVE_MOTOR;
import static frc.robot.Constants.FRONT_LEFT_MODULE_STEER_ENCODER;
import static frc.robot.Constants.FRONT_LEFT_MODULE_STEER_MOTOR;
import static frc.robot.Constants.FRONT_LEFT_MODULE_STEER_OFFSET;
import static frc.robot.Constants.FRONT_RIGHT_MODULE_DRIVE_MOTOR;
import static frc.robot.Constants.FRONT_RIGHT_MODULE_STEER_ENCODER;
import static frc.robot.Constants.FRONT_RIGHT_MODULE_STEER_MOTOR;
import static frc.robot.Constants.FRONT_RIGHT_MODULE_STEER_OFFSET;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.utilities.SwerveModule;
import frc.robot.utilities.SwerveModuleConstants;

import static frc.robot.Util.logf;

public class DrivetrainSubsystem extends SubsystemBase {

  // The maximum voltage that will be delivered to the drive motors.
  public static final double MAX_VOLTAGE = 12.0;
  // fixme Measure the drivetrain's maximum velocity or calculate the theoretical.
  // The formula for calculating the theoretical maximum velocity is:
  // <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> *
  // pi
  // By default this value is setup for a Mk3 standard module using Falcon500s to
  // drive.
  // An example of this constant for a Mk4 L2 module with NEOs to drive is:
  // 5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() *
  // SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
  /**
   * The maximum velocity of the robot in meters per second.
   * <p>
   * This is a measure of how fast the robot should be able to drive in a straight
   * line.
   */
  // TODO: this is a test config. The original values are below
  public static final double MAX_VELOCITY_METERS_PER_SECOND = 5880.0 / 60.0 / (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0) *
  0.10033 * Math.PI;
  /* 
  100.0 / 60.0 *
      (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0) *
      0.10033 * Math.PI;
*/
  // public static final double MAX_VELOCITY_METERS_PER_SECOND = 100.0 / 60.0 *
  // SdsModuleConfigurations.MK4I_L2.getDriveReduction() *
  // SdsModuleConfigurations.MK4I_L2.getWheelDiameter() * Math.PI;

  // public static final ModuleConfiguration MK4I_L2 = new ModuleConfiguration(
  // 0.10033,
  // (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0),
  // true,
  // (14.0 / 50.0) * (10.0 / 60.0),
  // false
  // );

  // public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
  // SdsModuleConfigurations.MK4I_L2.getDriveReduction() *
  // SdsModuleConfigurations.MK4I_L2.getWheelDiameter() * Math.PI;

  /**
   * The maximum angular velocity of the robot in radians per second.
   * <p>
   * This is a measure of how fast the robot can rotate in place.
   */
  // Here we calculate the theoretical maximum angular velocity. You can also
  // replace this with a measured amount.
  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
      Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

  public static final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      // Front left
      new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
      // Front right
      new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
      // Back left
      new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
      // Back right
      new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0));

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

  SwerveModule swerveModules[];

  private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

  public DrivetrainSubsystem() {
    // ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

    // There are 4 methods you can call to create your swerve modules.
    // The method you use depends on what motors you are using.
    //
    // Mk3SwerveModuleHelper.createFalcon500(...)
    // Your module has two Falcon 500s on it. One for steering and one for driving.
    //
    // Mk3SwerveModuleHelper.createNeo(...)
    // Your module has two NEOs on it. One for steering and one for driving.
    //
    // Mk3SwerveModuleHelper.createFalcon500Neo(...)
    // Your module has a Falcon 500 and a NEO on it. The Falcon 500 is for driving
    // and the NEO is for steering.
    //
    // Mk3SwerveModuleHelper.createNeoFalcon500(...)
    // Your module has a NEO and a Falcon 500 on it. The NEO is for driving and the
    // Falcon 500 is for steering.
    //
    // Similar helpers also exist for Mk4 modules using the Mk4SwerveModuleHelper
    // class.

    // By default we will use Falcon 500s in standard configuration. But if you use
    // a different configuration or motors
    // you MUST change it. If you do not, your code will crash on startup.
    // FIXed Setup motor configuration
    m_frontLeftModule = new SwerveModule(0, new SwerveModuleConstants(
        FRONT_LEFT_MODULE_DRIVE_MOTOR, FRONT_LEFT_MODULE_STEER_MOTOR, FRONT_LEFT_MODULE_STEER_ENCODER,
        FRONT_LEFT_MODULE_STEER_OFFSET));

    m_frontRightModule = new SwerveModule(1, new SwerveModuleConstants(
        FRONT_RIGHT_MODULE_DRIVE_MOTOR, FRONT_RIGHT_MODULE_STEER_MOTOR, FRONT_RIGHT_MODULE_STEER_ENCODER,
        FRONT_RIGHT_MODULE_STEER_OFFSET));

    m_backLeftModule = new SwerveModule(2, new SwerveModuleConstants(
        BACK_LEFT_MODULE_DRIVE_MOTOR, BACK_LEFT_MODULE_STEER_MOTOR, BACK_LEFT_MODULE_STEER_ENCODER,
        BACK_LEFT_MODULE_STEER_OFFSET));

    m_backRightModule = new SwerveModule(3, new SwerveModuleConstants(
        BACK_RIGHT_MODULE_DRIVE_MOTOR, BACK_RIGHT_MODULE_STEER_MOTOR, BACK_RIGHT_MODULE_STEER_ENCODER,
        BACK_RIGHT_MODULE_STEER_OFFSET));

    // m_frontLeftModule = Mk4iSwerveModuleHelper.createNeo(
    // // This parameter is optional, but will allow you to see the current state of
    // // the module on the dashboard.
    // tab.getLayout("Front Left Module", BuiltInLayouts.kList)
    // .withSize(2, 4)
    // .withPosition(0, 0),
    // // This can either be STANDARD or FAST depending on your gear configuration
    // Mk4iSwerveModuleHelper.GearRatio.L2,
    // // This is the ID of the drive motor
    // FRONT_LEFT_MODULE_DRIVE_MOTOR,
    // // This is the ID of the steer motor
    // FRONT_LEFT_MODULE_STEER_MOTOR,
    // // This is the ID of the steer encoder
    // FRONT_LEFT_MODULE_STEER_ENCODER,
    // // This is how much the steer encoder is offset from true zero (In our case,
    // // zero is facing straight forward)
    // FRONT_LEFT_MODULE_STEER_OFFSET);

    // // We will do the same for the other modules
    // m_frontRightModule = Mk4iSwerveModuleHelper.createNeo(
    // tab.getLayout("Front Right Module", BuiltInLayouts.kList)
    // .withSize(2, 4)
    // .withPosition(2, 0),
    // Mk4iSwerveModuleHelper.GearRatio.L2,
    // FRONT_RIGHT_MODULE_DRIVE_MOTOR,
    // FRONT_RIGHT_MODULE_STEER_MOTOR,
    // FRONT_RIGHT_MODULE_STEER_ENCODER,
    // FRONT_RIGHT_MODULE_STEER_OFFSET);

    // m_backLeftModule = Mk4iSwerveModuleHelper.createNeo(
    // tab.getLayout("Back Left Module", BuiltInLayouts.kList)
    // .withSize(2, 4)
    // .withPosition(4, 0),
    // Mk4iSwerveModuleHelper.GearRatio.L2,
    // BACK_LEFT_MODULE_DRIVE_MOTOR,
    // BACK_LEFT_MODULE_STEER_MOTOR,
    // BACK_LEFT_MODULE_STEER_ENCODER,
    // BACK_LEFT_MODULE_STEER_OFFSET);

    // m_backRightModule = Mk4iSwerveModuleHelper.createNeo(
    // tab.getLayout("Back Right Module", BuiltInLayouts.kList)
    // .withSize(2, 4)
    // .withPosition(6, 0),
    // Mk4iSwerveModuleHelper.GearRatio.L2,
    // BACK_RIGHT_MODULE_DRIVE_MOTOR,
    // BACK_RIGHT_MODULE_STEER_MOTOR,
    // BACK_RIGHT_MODULE_STEER_ENCODER,
    // BACK_RIGHT_MODULE_STEER_OFFSET);

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
    logf("zero Gyro DT\n");
    currentOrientation = 0;
    if (m_navx.isMagnetometerCalibrated()) {
      // // We will only get valid fused headings if the magnetometer is calibrated
      // System.out.println("returning the angle FUSE ZERO from the robot:
      // "+m_navx.getAngle());
      zeroNavx = m_navx.getFusedHeading();
    } else {
      zeroNavx = 0;
    }

    // m_navx.reset();
    m_navx.zeroYaw();
  }

  public void zeroGyroscope(double currentOrientation) {
    // FIXed Remove if you are using a Pigeon
    // m_pigeon.setFusedHeading(0.0);

    // FIXed Uncomment if you are using a NavX
    logf("zero Gyro\n");

    if (m_navx.isMagnetometerCalibrated()) {
      // // We will only get valid fused headings if the magnetometer is calibrated
      // System.out.println("returning the angle FUSE ZERO from the robot:
      // "+m_navx.getAngle());
      zeroNavx = m_navx.getFusedHeading();
    } else {
      zeroNavx = 0;
    }

    this.currentOrientation = currentOrientation;

    // m_navx.reset();
    m_navx.zeroYaw();
  }

  double currentOrientation = 0.0;
  double zeroNavx = 0.0;

  public Rotation2d getGyroscopeRotation() {
    // FIXed Remove if you are using a Pigeon
    // return Rotation2d.fromDegrees(m_pigeon.getFusedHeading());

    // FIXed Uncomment if you are using a NavX
    if (m_navx.isMagnetometerCalibrated()) {
      // // We will only get valid fused headings if the magnetometer is calibrated
      // System.out.println("returning the angle FUSE ZERO from the robot:
      // "+m_navx.getAngle());
      
      Rotation2d r = Rotation2d.fromDegrees(-(-m_navx.getFusedHeading() + zeroNavx + currentOrientation));
      SmartDashboard.putNumber("Rot Cal", r.getDegrees());
      return r;
    }
    //
    // We have to invert the angle of the NavX so that rotating the robot
    // counter-clockwise makes the angle increase.
    // return Rotation2d.fromDegrees(360.0 - m_navx.getYaw());

   Rotation2d r =  Rotation2d.fromDegrees(-(-m_navx.getYaw() + currentOrientation));
   SmartDashboard.putNumber("Rot NC", r.getDegrees());
   return r;
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

    // drive(m_chassisSpeeds.vyMetersPerSecond, m_chassisSpeeds.vxMetersPerSecond,
    // m_chassisSpeeds.omegaRadiansPerSecond);


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
    SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

    m_frontLeftModule.setDesiredState(states[0]);
    m_frontRightModule.setDesiredState(states[1]);
    m_backLeftModule.setDesiredState(states[2]);
    m_backRightModule.setDesiredState(states[3]);
    if (Robot.count % 20 == -7) { // disable Smartdash board display
      SmartDashboard.putNumber("FL", states[0].angle.getDegrees());
      SmartDashboard.putNumber("FR", states[1].angle.getDegrees());
      SmartDashboard.putNumber("BL", states[2].angle.getDegrees());
      SmartDashboard.putNumber("BR", states[3].angle.getDegrees());
      SmartDashboard.putNumber("Yaw", m_navx.getYaw());
      SmartDashboard.putNumber("Rot", getGyroscopeRotation().getDegrees());
    }
  }

  // private String chop(String s) {
  //   // returns the string after removing the last character
  //   return s.substring(0, s.length() - 1);
  // }

  public double getAngleForModule(SwerveModule module, SwerveModuleState state) {
    if (state.speedMetersPerSecond < 0.01) {
      return module.getAngle().getRadians();
    }
    return state.angle.getRadians();
  }

  public SwerveModuleState[] drive() {
    double fwd = m_chassisSpeeds.vxMetersPerSecond;
    double str = m_chassisSpeeds.vyMetersPerSecond;
    double rcw = m_chassisSpeeds.omegaRadiansPerSecond;
    // // if (str < JOYSTICK_THRESHOLD && fwd < JOYSTICK_THRESHOLD && rcw <
    // JOYSTICK_THRESHOLD) {
    // for (int i = 0; i < 4; ++i) {
    // wheels[i].setSpeed(0);
    // }
    // return;
    // }

    // first we transpose the direction vector with the angle of the gyro
    double robotAngle = 0; // getGyroscopeRotation().getRadians();
    // Math.toRadians(m_navx.getYaw()) + Math.PI / 4;

    double temp = fwd * Math.cos(robotAngle) + str * Math.sin(robotAngle);
    str = -fwd * Math.sin(robotAngle) + str * Math.cos(robotAngle);
    fwd = temp;

    double joystickAngle = getAngle(str, fwd);
    double joystickSpeed = getSpeed(str, fwd);
    double adjustedAngle = robotAngle + joystickAngle;
    str = Math.sin(adjustedAngle) * joystickSpeed;
    fwd = Math.cos(adjustedAngle) * joystickSpeed;

    double robotL = 30;
    double robotW = 30;

    // now we calculate the rotation and speed of each of the wheels
    double r = Math.sqrt(robotL * robotL + robotW * robotW);
    double a = str - (rcw * robotL / r);
    double b = str + (rcw * robotL / r);
    double c = fwd - (rcw * robotW / r);
    double d = fwd + (rcw * robotW / r);

    double speed[] = new double[] {
        getSpeed(b, c), // front left
        getSpeed(b, d), // front right
        getSpeed(a, c), // back left
        getSpeed(a, d) // back right
    };
    double maxSpeed = speed[0];
    for (int i = 1; i < 4; ++i) {
      if (maxSpeed < speed[i]) {
        maxSpeed = speed[i];
      }
    }
    for (int i = 0; i < 4; ++i) {
      speed[i] = maxSpeed > 1 ? speed[i] / maxSpeed : speed[i];
    }

    SwerveModuleState[] result = new SwerveModuleState[] {
        new SwerveModuleState(speed[0] // / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE
            , new Rotation2d(getAngle(b, d))),
        new SwerveModuleState(speed[1] // / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE
            , new Rotation2d(getAngle(b, c))),
        new SwerveModuleState(speed[2] // / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE
            , new Rotation2d(getAngle(a, d))),
        new SwerveModuleState(speed[3] // / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE
            , new Rotation2d(getAngle(a, c)))
    };

    // m_frontLeftModule.set(speed[0] / MAX_VELOCITY_METERS_PER_SECOND *
    // MAX_VOLTAGE, getAngle(b, d));
    // m_frontRightModule.set(speed[1] / MAX_VELOCITY_METERS_PER_SECOND *
    // MAX_VOLTAGE, getAngle(b, c));
    // m_backLeftModule.set(speed[2] / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
    // getAngle(a, d));
    // m_backRightModule.set(speed[3] / MAX_VELOCITY_METERS_PER_SECOND *
    // MAX_VOLTAGE, getAngle(a, c));
    if (Robot.count % 15 == 6) {
      // SmartDashboard.putNumber("FL", getAngle(b, d));
      // SmartDashboard.putNumber("FR", getAngle(b, c));
      // SmartDashboard.putNumber("BL", getAngle(a, d));
      // SmartDashboard.putNumber("BR", getAngle(a, c));
      // SmartDashboard.putNumber("Yaw", m_navx.getYaw());
    }

    // frontLeft.setAngle(getAngle(b, d));
    // frontRight.setAngle(getAngle(b, c));
    // backLeft.setAngle(getAngle(a, d));
    // backRight.setAngle(getAngle(a, c));
    // for (int i = 0; i < 4; ++i) {
    // wheels[i].setSpeed(maxSpeed > 1 ? speed[i] / maxSpeed : speed[i]);
    // }
    return result;
  }

  public void stop() {
    m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
  }

  public double getAngle(double x, double y) {
    return radians(90)
        - Math.atan2(y, x) + (Math.PI / 2);
  }

  public double getSpeed(double x, double y) { // }, double rotateX) {
    return Math.sqrt((x * x) + (y * y));
    // return Math.max(Math.max(Math.abs(x),Math.abs(y)), Math.abs(rotateX));
  }

  public double rotate(double x, double angle) {
    return x;
  }

  public static double radians(double degrees) {
    return Math.toRadians(degrees); // degrees * Math.PI / 180;
  }

  public static double degrees(double radians) {
    return Math.toDegrees(radians); // 180 * radians / Math.PI;
  }

  /**
   * Gets the current drivetrain position, as reported by the modules themselves.
   * 
   * @return current drivetrain state. Array orders are frontLeft, frontRight,
   *         backLeft, backRight
   */
  public SwerveModulePosition[] getModulePositions() {
    Rotation2d flA = m_frontLeftModule.getAngle(); // new Rotation2d(m_frontLeftModule.getAngle().getRadians());
                                                   // //getSteerAngle());
    Rotation2d frA = m_frontRightModule.getAngle(); // new Rotation2d(m_frontRightModule.getSteerAngle());
    Rotation2d blA = m_backLeftModule.getAngle(); // new Rotation2d(m_backLeftModule.getSteerAngle());
    Rotation2d brA = m_backRightModule.getAngle(); // new Rotation2d(m_backRightModule.getSteerAngle());
    double flP = m_frontLeftModule.getPosition();
    double frP = m_frontRightModule.getPosition();
    double blP = m_backLeftModule.getPosition();
    double brP = m_backRightModule.getPosition();

    // if (Robot.count % 20 == 6) {
    // SmartDashboard.putNumber("FL POS", flP);
    // SmartDashboard.putNumber("FR POS", frP);
    // SmartDashboard.putNumber("BR POS", brP);
    // SmartDashboard.putNumber("BL POS", blP);
    // 
    // The postition is in meters
    return new SwerveModulePosition[] {
        new SwerveModulePosition(flP, flA),
        new SwerveModulePosition(frP, frA),
        new SwerveModulePosition(blP, blA),
        new SwerveModulePosition(brP, brA)
    };
  }

}
