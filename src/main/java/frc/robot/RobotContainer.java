// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.robot.commands.RaiseArmCommand;
//command imports

//subsystem imports
import frc.robot.subsystems.ArmSubsystem;

public class RobotContainer {
  private final ArmSubsystem mArmSubsystem = new ArmSubsystem();

  // private final XboxController m_controller = new XboxController(0);
  private final CommandXboxController mController = new CommandXboxController(0); // Creates a CommandXboxController on
                                                                                  // port 1.

  // private final ArmSubsystem mArmSubsystemSubsystem = new ArmSubsystem();

  public RobotContainer() {
    SmartDashboard.putNumber("Arm Voltage", mArmSubsystem.getVoltage());
    SmartDashboard.putNumber("Arm Position (Radians)", mArmSubsystem.getMeasurement());
    configureButtonBindings();

  }

  private void configureButtonBindings() {

    // Run/Stop
    mController.a().onTrue(mArmSubsystem.runCommand());
    mController.b().onTrue(mArmSubsystem.stopCommand());

    mController.rightBumper().onTrue(mArmSubsystem.resetEncoder());

    // PID

    // mController.rightBumper().onTrue(mArmSubsystem.moveRotationsCommand(100));

    // mController.leftBumper().onTrue(mArmSubsystem.feedForwardCommand(100));

    mController.x().onTrue(Commands.runOnce(() -> {
      System.out.println("Running PID Command");
      mArmSubsystem.setGoal(Math.PI * 3);
      mArmSubsystem.enable();

    },
        mArmSubsystem));

    // mController.y().onTrue(mArmSubsystem.turnUntilLimitPressed());

    mController.y().onTrue(Commands.runOnce(() -> {
      System.out.println("Running PID Command");
      mArmSubsystem.setGoal(Math.PI * 1);
      mArmSubsystem.enable();
    },
        mArmSubsystem));

    // mController.leftStick().onTrue(mArmSubsystem.turnMoveCommand(mController.getLeftX()));

    // m_controller.x().onTrue(new moveRotationsCommand(mArmSubsystem));

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
    return mArmSubsystem.moveRotationsCommand(100);

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
