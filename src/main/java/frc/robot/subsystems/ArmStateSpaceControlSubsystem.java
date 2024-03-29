// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.math.estimator.KalmanFilter;

import frc.robot.Constants.ArmConstants;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;

import edu.wpi.first.math.util.Units;

import frc.robot.Constants.ArmConstants;

public class ArmStateSpaceControlSubsystem extends SubsystemBase {
  /** Creates a new ArmStateSpaceControlSubsystem. */

  private final CANSparkMax mMotor = new CANSparkMax(ArmConstants.kMotorPort, CANSparkLowLevel.MotorType.kBrushless);
  private final SparkMaxAlternateEncoder.Type kAltEncType = SparkMaxAlternateEncoder.Type.kQuadrature;

  private final int kCanID = 16;
  private final int kCPR = 8192;
  private final RelativeEncoder mAlternateEncoder = mMotor.getAlternateEncoder(kAltEncType, kCPR);

  // Moment of inertia of the arm, in kg * m^2. Can be estimated with CAD. If
  // finding this constant
  // is difficult, LinearSystem.identifyPositionSystem may be better.
  private static final double kArmMOI = 1.2;

  // Reduction between motors and encoder, as output over input. If the arm spins
  // slower than
  // the motors, this number should be greater than one.
  private static final double kArmGearing = 5.0;

  private static final double kDesiredPosition = Units.degreesToRadians(90.0);

  private final TrapezoidProfile m_profile = new TrapezoidProfile(
      new TrapezoidProfile.Constraints(
          Units.degreesToRadians(45),
          Units.degreesToRadians(90))); // Max arm speed and acceleration.
  private TrapezoidProfile.State m_lastProfiledReference = new TrapezoidProfile.State();

  // The plant holds a state-space model of our arm. This system has the following
  // properties:
  //
  // States: [position, velocity], in radians and radians per second.
  // Inputs (what we can "put in"): [voltage], in volts.
  // Outputs (what we can measure): [position], in radians.
  private final LinearSystem<N2, N1, N1> m_armPlant = LinearSystemId.createSingleJointedArmSystem(DCMotor.getNEO(2),
      kArmMOI, kArmGearing);

  // The observer fuses our encoder data and voltage inputs to reject noise.
  private final KalmanFilter<N2, N1, N1> m_observer = new KalmanFilter<>(
      Nat.N2(),
      Nat.N1(),
      m_armPlant,
      VecBuilder.fill(0.015, 0.17), // How accurate we
      // think our model is, in radians and radians/sec
      VecBuilder.fill(0.01), // How accurate we think our encoder position
      // data is. In this case we very highly trust our encoder position reading.
      0.020);

  // A LQR uses feedback to create voltage commands.
  private final LinearQuadraticRegulator<N2, N1, N1> m_controller = new LinearQuadraticRegulator<>(
      m_armPlant,
      VecBuilder.fill(Units.degreesToRadians(1.0), Units.degreesToRadians(10.0)), // qelms.
      // Position and velocity error tolerances, in radians and radians per second.
      // Decrease
      // this
      // to more heavily penalize state excursion, or make the controller behave more
      // aggressively. In this example we weight position much more highly than
      // velocity, but
      // this
      // can be tuned to balance the two.
      VecBuilder.fill(12.0), // relms. Control effort (voltage) tolerance. Decrease this to more
      // heavily penalize control effort, or make the controller less aggressive. 12
      // is a good
      // starting point because that is the (approximate) maximum voltage of a
      // battery.
      0.020); // Nominal time between loops. 0.020 for TimedRobot, but can be
  // lower if using notifiers.

  // The state-space loop combines a controller, observer, feedforward and plant
  // for easy control.
  private final LinearSystemLoop<N2, N1, N1> m_loop = new LinearSystemLoop<>(m_armPlant, m_controller, m_observer, 12.0,
      0.020);

  public ArmStateSpaceControlSubsystem() {
    mAlternateEncoder.setPositionConversionFactor(2 * Math.PI);
    mAlternateEncoder.setVelocityConversionFactor(2 * Math.PI);

    // Reset our loop to make sure it's in a known state.
    m_loop.reset(VecBuilder.fill(mAlternateEncoder.getPosition(), mAlternateEncoder.getVelocity()));

    // Reset our last reference to the current state.
    m_lastProfiledReference = new TrapezoidProfile.State(mAlternateEncoder.getPosition(),
        mAlternateEncoder.getVelocity());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command setGoal() {
    return run(() -> {
      TrapezoidProfile.State goal;
      goal = new TrapezoidProfile.State(kDesiredPosition, 0.0);

      // Step our TrapezoidalProfile forward 20ms and set it as our next reference
      m_lastProfiledReference = m_profile.calculate(0.020, m_lastProfiledReference, goal);
      m_loop.setNextR(m_lastProfiledReference.position, m_lastProfiledReference.velocity);
      // Correct our Kalman filter's state vector estimate with encoder data.
      m_loop.correct(VecBuilder.fill(mAlternateEncoder.getPosition()));

      // Update our LQR to generate new voltage commands and use the voltages to
      // predict the next
      // state with out Kalman filter.
      m_loop.predict(0.020);

      // Send the new calculated voltage to the motors.
      // voltage = duty cycle * battery voltage, so
      // duty cycle = voltage / battery voltage
      double nextVoltage = m_loop.getU(0);
      mMotor.setVoltage(nextVoltage);

    });

  }

}
