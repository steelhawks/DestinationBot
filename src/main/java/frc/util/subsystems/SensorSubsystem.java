/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.util.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

public abstract class SensorSubsystem extends Subsystem
{
  /** Enables the subsystem. */
  abstract public void enable();

  /** Disables the subsystem. */
  abstract public void disable();

  /** Sends a ping to the subsystem. */
  abstract public void ping();

  /** Checks if the subsystem is functioning properly. @return True if functioning */
  abstract public boolean isAlive();

  @Override
  public void initDefaultCommand() {}
}