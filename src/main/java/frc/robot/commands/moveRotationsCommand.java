// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class moveRotationsCommand extends Command {
  ArmSubsystem m_ArmSubsystem;
  double rotations;

  /** Creates a new moveRotationsCommand. */
  public moveRotationsCommand(ArmSubsystem m_armSubsystem) {

    this.m_ArmSubsystem = m_armSubsystem;
    // this.rotations = rotations;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.m_ArmSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("moveRotationsCommand initialized");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.print("Current position: ");
    System.out.println(m_ArmSubsystem.getEncoderPosition());

    if (m_ArmSubsystem.getEncoderPosition() < 20) {
      m_ArmSubsystem.setMotor(0.1);
    } else {
      m_ArmSubsystem.setMotor(0);
    }

  };

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ArmSubsystem.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
