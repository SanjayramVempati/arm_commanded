package frc.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

import frc.robot.Constants.ArmConstants;

import javax.print.attribute.standard.MediaSize.NA;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;

/** A robot arm subsystem that moves with a motion profile. */
public class ArmSubsystem extends ProfiledPIDSubsystem {
  private final CANSparkMax mMotor = new CANSparkMax(ArmConstants.kMotorPort, CANSparkLowLevel.MotorType.kBrushless);
  // private final CANSparkMax mFollowerMotor = new CANSparkMax(23,
  // CANSparkLowLevel.MotorType.kBrushless);

  private final SparkMaxAlternateEncoder.Type kAltEncType = SparkMaxAlternateEncoder.Type.kQuadrature;

  private final int kCanID = 16;
  private final int kCPR = 8192;
  private final RelativeEncoder mAlternateEncoder = mMotor.getAlternateEncoder(kAltEncType, kCPR);

  private final ArmFeedforward m_feedforward = new ArmFeedforward(
      ArmConstants.kSVolts, ArmConstants.kGVolts,
      ArmConstants.kVVoltSecondPerRad, ArmConstants.kAVoltSecondSquaredPerRad);

  private final DigitalInput mExtraLimitSwitch = new DigitalInput(2);
  private final DigitalInput mUpperLimitSwitch = new DigitalInput(1);
  private final DigitalInput mLowerLimitSwitch = new DigitalInput(0);

  /** Create a new ArmSubsystem. */
  public ArmSubsystem() {
    super(
        new ProfiledPIDController(
            ArmConstants.kP,
            0,
            0,
            new TrapezoidProfile.Constraints(
                ArmConstants.kMaxVelocityRadPerSecond,
                ArmConstants.kMaxAccelerationRadPerSecSquared)),
        0);
    mAlternateEncoder.setPositionConversionFactor(2 * Math.PI);

    // mFollowerMotor.follow(null);

    // Start arm at rest in neutral position
    // setGoal(ArmConstants.kArmOffsetRads);
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {

    // Calculate the feedforward from the setpoint

    if (mLowerLimitSwitch.get() & setpoint.velocity < 0) {
      return;

    }
    if (mUpperLimitSwitch.get() & setpoint.velocity > 0) {
      return;

    }
    double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
    System.out.println("Output " + output);
    System.out.println("Current Encoder Position: " + mAlternateEncoder.getPosition());

    // Add the feedforward to the PID output to get the motor output
    mMotor.setVoltage((output + feedforward) / 2);
  }

  @Override
  public double getMeasurement() {
    System.out.println(mAlternateEncoder.getPosition());
    return mAlternateEncoder.getPosition();
  }

  // Limit switch funcs
  public boolean isUpperLimitSwitchPressed() {
    return mUpperLimitSwitch.get();
  }

  public boolean isLowerLimitSwitchPressed() {
    return mLowerLimitSwitch.get();
  }

  // Motor funcs
  public void setMotor(double speed) {
    mMotor.set(speed);

  }

  public double getVoltage() {
    return mMotor.getBusVoltage();
  }

  public Command runCommand() {
    return runOnce(() -> setMotor(0.05));

  }

  // public Command runFollowerMotorCommand() {
  // return runOnce(() -> mFollowerMotor.set(.05));
  // }

  public Command stopCommand() {
    return runOnce(() -> mMotor.set(0));
  }

  public Command printEncoderPosition() {
    return runOnce(() -> System.out.println("Encoder position (radians) : " + mAlternateEncoder.getPosition()));
  }

  // Simple to pos
  public Command moveRotationsCommand(double radians) {
    return run(() -> {

      if (mAlternateEncoder.getPosition() < radians) {
        mMotor.set(0.05);
      } else {
        mMotor.set(0);
      }
    });
  }

  public Command feedForwardCommand(double rad) {
    return runOnce(() -> {
      System.out.print("Going to " + rad + " radians");
      setGoal(rad);
      enable();

    });
  }

  public Command resetEncoder() {
    return runOnce(() -> {
      mAlternateEncoder.setPosition(0);
      // mFollowerAlternateEncoder.setPosition(0);
    });
  }

  public boolean isFinished() {
    return true;
  }

  @Override
  public void periodic() {
    // TODO Auto-generated method stub
    super.periodic();
    System.out.println(mAlternateEncoder.getPosition());

  }

  public Command turnUntilLimitPressed() {
    return run(() -> {
      if (mLowerLimitSwitch.get() | mUpperLimitSwitch.get() | mExtraLimitSwitch.get()) {
        mMotor.set(0);
      } else {
        mMotor.set(0.1);
      }
    });
  }

  public Command turnMoveCommand(double val) {
    return run(() -> {
      mMotor.set(val * 0.1);
      System.out.println(val);
    });
  }

}