// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

public class ArmPIDSubsystem extends PIDSubsystem {

  // --- Constants ---
  private final int kCanID;
  private final SparkMaxAlternateEncoder.Type kAltEncType;
  private int kCPR;

  // Components of the Arm
  private final CANSparkMax m_motor;

  private final RelativeEncoder m_alternateEncoder;

  double encoderPosition;

  private final DigitalInput lowerLimitSwitch;
  private final DigitalInput upperLimitSwitch;

  // Feedforward
  private final SimpleMotorFeedforward m_feedForward;

  /** Creates a new ArmPIDSubsystem. */
  public ArmPIDSubsystem() {

    super(
        // The PIDController used by the subsystem
        new PIDController(0, 0, 0));

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

    m_feedForward = new SimpleMotorFeedforward(kCPR, encoderPosition);

  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
    double feedForward = m_feedForward.calculate(setpoint);
    m_motor.set(output + feedForward);
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return encoderPosition;
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

  public Command setSetpoint() {
    return runOnce(() -> useOutput(kCPR, 2));
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

}
