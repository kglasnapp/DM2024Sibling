package frc.robot.utilities;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.utilities.CANSparkMaxUtil.Usage;

public class SwerveModule {
  public static final double TURBO = 1;
  public static final double NORMAL = 5;
  public static final double PRECISION = 10;

  private static double powerRatio = SwerveModule.TURBO;

  public static final double wheelDiameter = Units.inchesToMeters(4.0);
  public static final double wheelCircumference = wheelDiameter * Math.PI;

  public static final double openLoopRamp = 0.25;
  public static final double closedLoopRamp = 0.0;

  /* Swerve Voltage Compensation */
  public static final double voltageComp = 12.0;

  /* Swerve Current Limiting */
  public static final int angleContinuousCurrentLimit = 20;
  public static final int driveContinuousCurrentLimit = 60;

  /* Angle Motor PID Values */
  public static final double angleKP = 0.01;
  public static final double angleKI = 0.0;
  public static final double angleKD = 0.0;
  public static final double angleKFF = 0.0;

  /* Drive Motor PID Values */
  public static final double driveKP = 0.1;
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
  public static final boolean driveInvert = true;

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

    driveConversionPositionFactor = (wheelDiameter * Math.PI) * swerve_type.driveGearRatio;
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

  public static double getPowerRatio() {
    if (RobotContainer.driveHID.getLeftBumper() ||
        RobotContainer.driveHID.getLeftTriggerAxis() > 0.1 ||
        DrivetrainSubsystem.speedsComeFromController) {
      return 600;
    }
    return powerRatio;
    // return 600;
  }

  public static void setPowerRatio(double powerRatio) {
    SwerveModule.powerRatio = powerRatio;
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    // Custom optimize command, since default WPILib optimize assumes continuous
    // controller which
    // REV and CTRE are not
    desiredState = OnboardModuleState.optimize(desiredState, getState().angle);
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

    angleController.setFeedbackDevice(integratedAngleEncoder);

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
    driveController.setP(angleKP);
    driveController.setI(angleKI);
    driveController.setD(angleKD);
    driveController.setFF(angleKFF);
    driveMotor.enableVoltageCompensation(voltageComp);
    driveMotor.burnFlash();
    driveEncoder.setPosition(0.0);
  }

  private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
    if (isOpenLoop) {
      double percentOutput = desiredState.speedMetersPerSecond / DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND;
      driveMotor.set(percentOutput / 20);
    } else {
      double desiredSpeed = desiredState.speedMetersPerSecond / getPowerRatio();
      // desiredSpeed = desiredState.speedMetersPerSecond;
      driveController.setReference(
          desiredSpeed,
          ControlType.kVelocity,
          0,
          feedforward.calculate(desiredSpeed));
    }
  }

  int resetIteration = 0;

  private void setAngle(SwerveModuleState desiredState) {
    double desiredAngle = desiredState.angle.getDegrees();
    // double currentAngle = integratedAngleEncoder.getPosition();
    if (integratedAngleEncoder.getVelocity() < 0.5) {
      if (++resetIteration >= 500) {
        resetIteration = 0;

        double absoluteAngle = getCanCoder().getDegrees();
        integratedAngleEncoder.setPosition(absoluteAngle % 360);
        // currentAngle = absoluteAngle;
        // System.out.println("***** Reset Module "+moduleNumber+" angleMotorId =
        // "+angleMotor.getDeviceId()+" absolute angle = "+absoluteAngle+ " integrated
        // angle = "+integratedAngleEncoder.getPosition());
      }
    } else {
      resetIteration = 0;
    }

    // Prevent rotating module if speed is less then 1%. Prevents jittering.
    // Rotation2d angle =
    // (Math.abs(desiredState.speedMetersPerSecond) <= ( 0.01))
    // ? lastAngle
    // : desiredState.angle;
    // double currentAngleMod = currentAngle % (360);
    // if (currentAngleMod < 0.0) {
    // currentAngleMod += 360;
    // }
    // double adjustedReferenceAngle = desiredAngle + currentAngle -
    // currentAngleMod;
    // // if (desiredState.angle.getDegrees() - currentAngleMod > 180) {
    // // adjustedReferenceAngle -= 360;
    // // } else if (desiredAngle - currentAngleMod < -180) {
    // // adjustedReferenceAngle += 360;
    // // }

    double angle = desiredAngle; // + (currentAngle - currentAngle % 360);

    if (Robot.count % 20 == 0) {

      // System.out.println("Module "+moduleNumber+" setting angle
      // "+desiredState.angle.getDegrees());
    }

    angleController.setReference(angle, CANSparkBase.ControlType.kPosition);

    if (Robot.count % 10 == 0) {
      double cancoderAngle = integratedAngleEncoder.getPosition();
      if (Math.abs(angle - cancoderAngle) > 2.5) {
        // Util.logf("Module %d set point:%.3f angle %.3f\n ", moduleNumber, angle,
        // integratedAngleEncoder.getPosition());
      }
    }
    // lastAngle = Rotation2d.fromDegrees(angle);
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