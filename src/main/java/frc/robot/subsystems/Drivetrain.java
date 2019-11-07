/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Robot;
import frc.robot.commands.DiffDrive;
import frc.util.subsystems.MechanicalSubsystem;

public class Drivetrain extends MechanicalSubsystem 
{
  //SPARK MAX MOTORS
  private final WPI_TalonSRX m_leftOne, m_leftTwo, m_leftThree, m_rightOne, m_rightTwo, m_rightThree;

  //SPEED CONTROLLER GROUPS
  public final SpeedControllerGroup m_leftGroup, m_rightGroup;

  //DIFFERENTIAL DRIVE
  private final DifferentialDrive diffDrive;

  //SHIFTING SOLENOIDS
  private final DoubleSolenoid sol_shift;

  //NAVX MXP gyro
  public final AHRS gyro;

  //GRAYHILL OPTICAL ENCODERS
  public Encoder enc_leftO, enc_rightO;

  /** Drivetrain constructor. */
  public Drivetrain() 
  {
    //SPARK MAX LEFT MOTORS
    this.m_leftOne = new WPI_TalonSRX(Robot.ROBOTMAP.m_leftOne);
    this.m_leftTwo = new WPI_TalonSRX(Robot.ROBOTMAP.m_leftTwo);
    this.m_leftThree = new WPI_TalonSRX(Robot.ROBOTMAP.m_leftThree);
    
    //SPARK MAX RIGHT MOTORS
    this.m_rightOne = new WPI_TalonSRX(Robot.ROBOTMAP.m_rightOne);
    this.m_rightTwo = new WPI_TalonSRX(Robot.ROBOTMAP.m_rightTwo);
    this.m_rightThree = new WPI_TalonSRX(Robot.ROBOTMAP.m_rightThree);

    //SPEED CONTROLLER GROUPS
    this.m_leftGroup = new SpeedControllerGroup(this.m_leftOne, this.m_leftTwo, this.m_leftThree);
    this.m_rightGroup = new SpeedControllerGroup(this.m_rightOne, this.m_rightTwo, this.m_rightThree);

    //DIFFERENTIAL DRIVE
    this.diffDrive = new DifferentialDrive(this.m_leftGroup, this.m_rightGroup);

    //NAVX MXP GYRO
    this.gyro = new AHRS(SPI.Port.kMXP);

    //SHIFTING SOLENOIDS
    this.sol_shift = new DoubleSolenoid(Robot.ROBOTMAP.sol_shiftOn, Robot.ROBOTMAP.sol_shiftOff);

    //GRAYHILL OPTICAL ENCODERS
    this.enc_leftO = new Encoder(Robot.ROBOTMAP.enc_leftA, Robot.ROBOTMAP.enc_leftB, false, EncodingType.k4X);
    this.enc_rightO = new Encoder(Robot.ROBOTMAP.enc_rightA, Robot.ROBOTMAP.enc_rightB, false, EncodingType.k4X);

    //SET ROBOT TO LOW GEAR
    sol_shift.set(DoubleSolenoid.Value.kReverse);
  }

  @Override
  public void initDefaultCommand() 
  {
    setDefaultCommand(new DiffDrive());
  }

  /** Drives the robot with the arcadeDrive() method for differental drive robots. */
  public void arcadeDrive(Joystick stick) 
  {
    this.diffDrive.arcadeDrive(stick.getY(), -stick.getTwist());
  }

  /** Toggles between high and low gear. */
  public void shiftGear() 
  {
    if(sol_shift.get() == DoubleSolenoid.Value.kForward) 
    {
      this.sol_shift.set(DoubleSolenoid.Value.kReverse);
    } 
    else 
    {
      this.sol_shift.set(DoubleSolenoid.Value.kForward);
    }
  }

  /** Rotates the robot. */
  public void rotate(double speed)
  {
    this.m_leftGroup.set(speed);
    this.m_rightGroup.set(speed);
  }

  public boolean stop()
  {
    rotate(0);
    return true;
  }

  public void ping() {}

  public boolean isAlive()
  {
    return this.diffDrive.isAlive();
  }

  /** Gets the left encoder position @return left encoder position */
  public double getLeftEncPosition() 
  {
    return this.enc_leftO.getRaw();
  }

  /** Gets the right encoder position @return right encoder position */
  public double getRightEncPosition() 
  {
    return this.enc_rightO.getRaw();
  }

  /**
   * Reset the robot gyro.
   */
  public void resetGyro() 
  {
    this.gyro.reset();
    this.gyro.zeroYaw();
  }
}