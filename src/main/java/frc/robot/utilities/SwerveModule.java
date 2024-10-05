package frc.robot.utilities;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.utilities.CANSparkMaxUtil.Usage;

public class SwerveModule {
  public static final double TURBO = 1;
  public static final double NORMAL = 5;
  public static final double PRECISION = 10;
  public static final double DEFAULT = TURBO;

  private static double powerRatio = DEFAULT;

  public static final double openLoopRamp = 0.25;
  public static final double closedLoopRamp = 0.0;

  /* Swerve Voltage Compensation */
  public static final double voltageComp = 12.0;

  /* Swerve Current Limiting */
  public static final int angleContinuousCurrentLimit = 20;
  public static final int driveContinuousCurrentLimit = 60;

  /* Angle Motor PID Values */
  public static final double angleKP = 0.012;
  public static final double angleKI = 0.0006;
  public static final double angleKD = 0.0;
  public static final double angleKFF = 0.0;

  /* Drive Motor PID Values */
  public static final double driveKP = 0.01;
  public static final double driveKI = 0.0;
  public static final double driveKD = 0.0;
  public static final double driveKFF = 0.0;

  /* Drive Motor Characterization Values */
  public static final double driveKS = 0.667;
  public static final double driveKV = 2.44;
  public static final double driveKA = 0.27;

  /* Neutral Modes */
  public static final IdleMode angleNeutralMode = IdleMode.kBrake;
  public static final IdleMode driveNeutralMode = IdleMode.kBrake;

  /* Motor Inverts */
  public static final boolean driveInvert = false;

  public static final boolean angleInvert = true;

  /* Angle Encoder Invert */
  public static final boolean canCoderInvert = false;

  public int moduleNumber;

  private CANSparkMax angleMotor;
  private CANSparkMax driveMotor;

  private RelativeEncoder driveEncoder;
  private RelativeEncoder integratedAngleEncoder;
  private CANcoder angleEncoder;

  private final SparkPIDController driveController;
  private final SparkPIDController angleController;

  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
      driveKS, driveKV, driveKA);

  /* Drive Motor Conversion Factors */
  public final double driveConversionPositionFactor;
  public final double driveConversionVelocityFactor;
  public final double angleConversionFactor;

  public SwerveModule(int moduleNumber, SwerveModuleType swerve_type, SwerveModuleIds moduleConstants) {
    this.moduleNumber = moduleNumber;

    driveConversionPositionFactor = Constants.WHEEL_CIRCUMFERENCE * swerve_type.driveGearRatio;
    driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
    angleConversionFactor = 360.0 * swerve_type.angleGearRatio;

    /* Angle Encoder Config */
    angleEncoder = new CANcoder(moduleConstants.cancoderID);
    configAngleEncoder();

    /* Angle Motor Config */
    angleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
    integratedAngleEncoder = angleMotor.getEncoder();
    angleController = angleMotor.getPIDController();
    configAngleMotor();

    /* Drive Motor Config */
    driveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
    driveEncoder = driveMotor.getEncoder();
    driveController = driveMotor.getPIDController();
    configDriveMotor();

    // lastAngle = getState().angle;
  }

  // FIXME: This should prob be in drivetrain subsystem
  public static double getPowerRatio() {
    return powerRatio;
  }

  // FIXME: This should prob be in drivetrain subsystem
  public static void setPowerRatio(double powerRatio) {
    SwerveModule.powerRatio = powerRatio;
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    // Custom optimize command, since default WPILib optimize assumes continuous
    // controller which
    // REV and CTRE are not
    desiredState = SwerveModuleState.optimize(desiredState, getAngle());
    setAngle(desiredState);
    setSpeed(desiredState, false);
  }

  double absolutePosition = 0;

  private void resetToAbsolute() {
    absolutePosition = getCanCoder().getDegrees();// + angleOffset.getDegrees();
    // System.out.println("Module " + moduleNumber + " absolute angle = " +
    // absolutePosition
    // + " getCanCoder().getDegrees() - angleOffset.getDegrees() " +
    // getCanCoder().getDegrees() + " - "
    // + angleOffset.getDegrees());
    integratedAngleEncoder.setPosition(absolutePosition % 360);
  }

  private void configAngleEncoder() {
    // angleEncoder.configFactoryDefault();
    // CANCoderUtil.setCANCoderBusUsage(angleEncoder, CCUsage.kMinimal);
    // angleEncoder.configAllSettings(ctreConfigs.swerveCanCoderConfig);
  }

  private void configAngleMotor() {
    angleMotor.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(angleMotor, Usage.kPositionOnly);
    angleMotor.setSmartCurrentLimit(angleContinuousCurrentLimit);
    angleMotor.setInverted(angleInvert);
    angleMotor.setIdleMode(angleNeutralMode);
    integratedAngleEncoder.setPositionConversionFactor(angleConversionFactor);
    integratedAngleEncoder.setVelocityConversionFactor(angleConversionFactor / 60.0);
    angleController.setP(angleKP);
    angleController.setI(angleKI);
    angleController.setD(angleKD);
    angleController.setFF(angleKFF);
    angleController.setIMaxAccum(20, 0);
    angleController.setIZone(2.0);

    angleController.setFeedbackDevice(integratedAngleEncoder);
    angleController.setPositionPIDWrappingEnabled(true);
    angleController.setPositionPIDWrappingMinInput(-180);
    angleController.setPositionPIDWrappingMaxInput(180);

    angleMotor.enableVoltageCompensation(voltageComp);
    angleMotor.burnFlash();
    resetToAbsolute();
  }

  private void configDriveMotor() {
    driveMotor.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(driveMotor, Usage.kAll);
    driveMotor.setSmartCurrentLimit(driveContinuousCurrentLimit);
    driveMotor.setInverted(driveInvert);
    driveMotor.setIdleMode(driveNeutralMode);
    driveEncoder.setVelocityConversionFactor(driveConversionVelocityFactor);
    driveEncoder.setPositionConversionFactor(driveConversionPositionFactor);
    driveController.setP(driveKP);
    driveController.setI(driveKI);
    driveController.setD(driveKD);
    driveController.setFF(driveKFF);
    driveMotor.enableVoltageCompensation(voltageComp);
    driveMotor.burnFlash();
    driveEncoder.setPosition(0.0);
  }

  private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
    if (isOpenLoop) {
      // FIXME: Do we even need this?
      double percentOutput = desiredState.speedMetersPerSecond / Constants.MAX_VELOCITY_METERS_PER_SECOND;
      driveMotor.set(percentOutput / 20);
    } else {
      double desiredSpeed = desiredState.speedMetersPerSecond / getPowerRatio();
      // desiredSpeed = desiredState.speedMetersPerSecond;
      driveController.setReference(
          desiredSpeed,
          ControlType.kVelocity,
          0,
          // TODO: Investigate what this does
          feedforward.calculate(desiredSpeed));
    }
  }

  int resetIteration = 0;

  private void setAngle(SwerveModuleState desiredState) {
    // Sync relative encoder with absolute encoder every 5 seconds
    if (integratedAngleEncoder.getVelocity() < 0.5) {
      if (++resetIteration >= 250) {
        resetIteration = 0;

        double absoluteAngle = getCanCoder().getDegrees();
        integratedAngleEncoder.setPosition(absoluteAngle % 360);
      }
    } else {
      resetIteration = 0;
    }

    double desiredAngle = desiredState.angle.getDegrees();
    angleController.setReference(desiredAngle, CANSparkBase.ControlType.kPosition);
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromDegrees(integratedAngleEncoder.getPosition());
  }

  public Rotation2d getCanCoder() {
    double rotations = angleEncoder.getAbsolutePosition().getValue();
    return Rotation2d.fromDegrees(360 * rotations);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(driveEncoder.getVelocity(), getAngle());
  }

  public double getPosition() {
    return driveEncoder.getPosition();
  }

  public double getCurrent() {
    return driveMotor.getOutputCurrent();
  }

  public double getVoltage() {
    return driveMotor.getBusVoltage();
  }

  public double getVelocity() {
    return driveMotor.getEncoder().getVelocity();
  }
}