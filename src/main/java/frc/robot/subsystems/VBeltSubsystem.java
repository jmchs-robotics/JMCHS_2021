/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VBeltMotors;
import frc.robot.commands.DefaultIntakeCommand;

public class VBeltSubsystem extends SubsystemBase {
  private DoubleSolenoid m_solenoid;
  private VictorSPX m_motor_right;
  private VictorSPX m_motor_left;

  private boolean m_lowered;
  private double m_forwardSpeed;
  private double m_reverseSpeed;
  private double m_reversePulse;
  private XboxController m_secondaryController = new XboxController(2);
  public static int m_int = 0;

  /**
   * Creates a new IntakeSubsystem.
   */
  public VBeltSubsystem() {
    //m_solenoid = new DoubleSolenoid(IntakeActuators.intakeSoleniodForward, IntakeActuators.intakeSoleniodBackward);
    m_motor_right = new VictorSPX(VBeltMotors.VBeltMotorRightID);
    m_motor_left = new VictorSPX(VBeltMotors.VBeltMotorLeftID);
    //m_forwardSpeed = IntakeActuators.forwardSpeed;
    //m_reverseSpeed = IntakeActuators.reverseSpeed;
    //m_reversePulse = IntakeActuators.reversePulse;
    SmartDashboard.putNumber("class created", m_int);

    /** if (IntakeActuators.TUNE){
      SmartDashboard.putNumber("Intake Motor Output Percent", m_motor.getMotorOutputPercent());
      SmartDashboard.putBoolean("Intake Lowered????", isLowered());
      SmartDashboard.putNumber("Intake Motor Forward Speed", m_forwardSpeed);
      SmartDashboard.putNumber("Intake Motor Reverse Speed", m_reverseSpeed);
      SmartDashboard.putNumber("Intake Motor Reverse Pulse Time", m_reversePulse);
    } */
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    /**if (IntakeActuators.TUNE){
      double fs, rs, rp;
      SmartDashboard.putNumber("Intake Motor Output Percent", m_motor.getMotorOutputPercent());
      SmartDashboard.putBoolean("Intake Lowered????", isLowered());
      fs = SmartDashboard.getNumber("Intake Motor Forward Speed", 0);
      rs = SmartDashboard.getNumber("Intake Motor Reverse Speed", 0);
      rp = SmartDashboard.getNumber("Intake Motor Reverse Pulse Time", 0);

      if( fs != m_forwardSpeed) {
        m_forwardSpeed = fs;
          setMotor(fs);
      }
      if( rs != m_reverseSpeed) {
        m_reverseSpeed = rs;
          setMotor(rs);
      }
      if( rp != m_reversePulse) {
        m_reversePulse = rp;
      }
      
    } */
  }

  /**
   * Extends the intake piston.
   */
  public void lowerIntake(){
    m_solenoid.set(Value.kForward);
    m_lowered = true;
  }

  /**
   * Retracts the intake piston.
   */
  public void raiseIntake(){
    m_solenoid.set(Value.kReverse);
    m_lowered = false;
  }

  /**
   * Turns off the intake solenoid.
   */
  public void turnOffSolenoid(){
    m_solenoid.set(Value.kOff);
  }

  /**
   * Get the status of the intake
   * @return True if the intake is lowered
   */
  public boolean isLowered() {
    return m_lowered; // return true if the intake piston is forward (intake is lowered down)
  }

  /**
   * Set the intake motor to a speed between -1 and 1.
   * @param speed
   */
  public void setMotor(double speedRight, double speedLeft){
    m_motor_right.set(ControlMode.PercentOutput,speedRight);
    m_motor_left.set(ControlMode.PercentOutput,speedLeft);
  }

  public void stopMotor (){
    m_motor_right.set(ControlMode.PercentOutput, 0.0);
    m_motor_left.set(ControlMode.PercentOutput, 0.0);
  }

  public void motorForward() {
    setMotor(m_forwardSpeed, m_forwardSpeed);
  }

  public void motorReverse() {
    setMotor( m_reverseSpeed, m_reverseSpeed);
  }

  public double getReversePulse() {
    return m_reversePulse;
  }

  public double getReverseSpeed() {
    return m_reverseSpeed;
  }
}
