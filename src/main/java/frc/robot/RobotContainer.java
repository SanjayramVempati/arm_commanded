// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import frc.robot.commands.RaiseArmCommand;
//command imports
import frc.robot.commands.moveRotationsCommand;

//subsystem imports
import frc.robot.subsystems.ArmPIDSubsystem;

public class RobotContainer {
  private final ArmPIDSubsystem m_arm = new ArmPIDSubsystem();

  // private final XboxController m_controller = new XboxController(0);
  private final CommandXboxController m_controller = new CommandXboxController(0); // Creates a CommandXboxController on
                                                                                   // port 1.

  // private final ArmSubsystem m_armSubsystem = new ArmSubsystem();

  public RobotContainer() {

    configureButtonBindings();

  }

  private void configureButtonBindings() {

    // Run/Stop
    m_controller.a().onTrue(m_arm.runCommand());
    m_controller.b().onTrue(m_arm.stopCommand());

    m_controller.y().onTrue(m_arm.resetEncoder());

    // PID

    m_controller.rightBumper().onTrue(m_arm.moveRotationsCommandLambda(10));

    m_controller.leftBumper().onTrue(m_arm.setSetpoint());
    // m_controller.x().onTrue(new moveRotationsCommand(m_arm));

  }

  /**
   * 
   * 
   * /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   * 
   */
  public Command getAutonomousCommand() {

    // return new PlatformDockPidCommand_X(m_drivetrainSubsystem);
    // return new RaiseArmCommand();
    return m_arm.moveRotationsCommandLambda(0);

  }

  // deadband and modifyAxis methods
  /*
   * private static double deadband(double value, double deadband) {
   * if (Math.abs(value) > deadband) {
   * if (value > 0.0) {
   * return (value - deadband) / (1.0 - deadband);
   * } else {
   * return (value + deadband) / (1.0 - deadband);
   * }
   * } else {
   * return 0.0;
   * }
   * }
   * 
   * private static double modifyAxis(double value) {
   * // Deadband
   * value = deadband(value, 0.05); // sanjay change: Deadband value before 0.05
   * 
   * // Square the axis
   * value = Math.copySign(value * value, value);
   * 
   * return value;
   * }
   */

}
