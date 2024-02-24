package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj2.command.Command;

public class ArmSubsystem extends SubsystemBase {
  // Variables

  // // Settings
  // private static final double MIN_ROTATIONS = 0.0;
  // private static final double MAX_ROTATIONS = 1.0;

  // Array of Angle Positions

  // Constants
  private final int kCanID;
  private final SparkMaxAlternateEncoder.Type kAltEncType;
  private int kCPR;

  // Components of the Arm
  private final CANSparkMax m_motor;

  private final RelativeEncoder m_alternateEncoder;

  double encoderPosition;

  private final DigitalInput lowerLimitSwitch;
  private final DigitalInput upperLimitSwitch;

  // PID
  private final SimpleMotorFeedforward feedforward;

  /// Arm component of the robot
  public ArmSubsystem() {

    kCanID = 16;
    this.kAltEncType = SparkMaxAlternateEncoder.Type.kQuadrature;
    kCPR = 8192;

    m_motor = new CANSparkMax(kCanID, CANSparkLowLevel.MotorType.kBrushless);
    System.out.println("Created sparkmax");
    m_motor.restoreFactoryDefaults();

    m_alternateEncoder = m_motor.getAlternateEncoder(kAltEncType, kCPR);
    m_alternateEncoder.setPosition(0.0);

    encoderPosition = m_alternateEncoder.getPosition();

    lowerLimitSwitch = new DigitalInput(0);
    upperLimitSwitch = new DigitalInput(1);

    feedforward = new SimpleMotorFeedforward(kCPR, encoderPosition);
  }

  // motor funcs
  public void setMotor(double speed) {
    m_motor.set(speed);
  }

  public void stopMotor() {
    m_motor.stopMotor();
  }

  // limit switch funcs
  public boolean isUpperSwitchPressed() {
    return lowerLimitSwitch.get();
  }

  public boolean isLowerSwitchPressed() {
    return upperLimitSwitch.get();
  }

  // encoder funcs
  public double getEncoderPosition() {
    return m_alternateEncoder.getPosition();
  }

  public void setEncoderPosition(double position) {
    m_alternateEncoder.setPosition(position);
  }

  // Arm funcs

  public void raiseArm() {
    if (!isUpperSwitchPressed()) {
      setMotor(0.5);
    } else {
      stopMotor();
      setEncoderPosition(0);
    }
  }

  public void lowerArm() {
    if (!isLowerSwitchPressed()) {
      setMotor(-0.5);
    } else {
      stopMotor();
      setEncoderPosition(0);
    }
  }

  // move to position
  public void moveToPosition(double position) {
    if (position > getEncoderPosition() && !isUpperSwitchPressed()) {
      m_motor.set(0.05);
    } else {
      m_motor.set(0);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    System.out.print("Current encoder position:   ");
    System.out.println(getEncoderPosition());
  }

  // --- COMMANDS ---

  public Command runCommand() {
    return runOnce(() -> m_motor.set(0.1));

  }

  public Command printEncoderPosition() {
    return runOnce(() -> m_alternateEncoder.getPosition());
  }

  public Command resetEncoder() {
    return runOnce(() -> m_alternateEncoder.setPosition(0));
  }

  public Command stopCommand() {
    return runOnce(() -> m_motor.set(0));
  }

  public Command moveRotationsCommandLambda(double rotations) {
    return run(() -> {

      if (getEncoderPosition() < rotations) {
        setMotor(0.05);
      } else {
        setMotor(0);
      }
    });
  }

  public Command moveRotationsPID(double rotations) {
    double Kp = 0.005; // P gain (may be tuned)
    double currentPosition = getEncoderPosition(); // Get current encoder angle
    double desiredPosition = rotations; // Desired encoder angle
    double error = desiredPosition - currentPosition; // Calculate error
    double command = error * Kp; // Error times P = P command

    return run(() -> {
      setMotor(command);
    });
  }

}
