/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.OI;
import frc.robot.RobotMap;

import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

/**
 * Add your docs here.
 */
public class CargoPIDSubsystem extends PIDSubsystem {
  
  private WPI_TalonSRX cargoArm = null;
  
  public static double lastManualPos = 95;

  /**
   * poitive = toward bumper
   */
  static double[][] manualPowerTable = {
    //pot position, max power going towards hatch, max power going towards bumper
    {0, 0.0, 0.3},
    {210, 0.25, 0.15},
    {310, 0.3, 0.0},
    {10000, 0.0, 0.0}
  };

  static double[][] PIDPowerTable = {
    {0, 0.0, 0.4},
    {210, 0.25, 0.15},
    {260, 0.25, 0.15},
    {310, 0.4, 0.0},
    {10000, 0.0, 0.0}
  };

  static double kP = 0.015;
  static double kI = 0.0001;
  static double kD = 0.001;

  /**
   * Add your docs here.
   */
  public CargoPIDSubsystem() {
    // Intert a subsystem name and PID values here
    
    super("Cargo Arm", kP, kI, kD);
    SmartDashboard.putNumber("kP", kP);
    SmartDashboard.putNumber("kI", kI);
    SmartDashboard.putNumber("kD", kD);

    cargoArm = new WPI_TalonSRX(1);
    cargoArm.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 30);
    cargoArm.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    cargoArm.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    
    //empirically derived -- is it straight amps or a percentage???
    // cargoArm.configPeakCurrentLimit(18, 30);
    // cargoArm.configContinuousCurrentLimit(15, 30);

    setAbsoluteTolerance(5);
    disable();
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  @Override
  protected double returnPIDInput() {
    double pos = getPosition();
    int colRef = getPIDError() > 0 ? 2 : 1;
    double maxPower = getMaxPower(pos, colRef, PIDPowerTable);
    setOutputRange(-maxPower, maxPower);
    SmartDashboard.putNumber("Pot Position", pos);
    SmartDashboard.putNumber("Col Ref", colRef);
    SmartDashboard.putNumber("Max Power", maxPower);
    return pos;
    // return getPosition();
  }

  @Override
  protected void usePIDOutput(double output) {
    //green is toward bumper
    SmartDashboard.putNumber("PID Setpoint", getSetpoint());
    SmartDashboard.putNumber("Pot Position", getPosition());
    SmartDashboard.putNumber("PID Output", output);
    cargoArm.set(ControlMode.PercentOutput, output);
  }

  @Override
  public double getPosition() {
    return cargoArm.getSelectedSensorPosition();
  }

  public void changeSetpoint(double setpoint) {
    setSetpoint(setpoint);
    enable();
  }

  public void manualCargoArm(OI oi) {
    double joyVal = deadzone(oi.getJoystick().getRawAxis(RobotMap.RIGHT_JOYSTICK_Y));
    int colRef = joyVal > 0 ? 1 : 2;
    double scaleFactor = getMaxPower(cargoArm.getSelectedSensorPosition(), colRef, manualPowerTable);

    /**
     * negative because the motor and the arm drive in opposite directions
     */
    double motorOutput = -joyVal * scaleFactor;

    if(joyVal == 0) {
      motorOutput = 0;
    }

    lastManualPos = cargoArm.getSelectedSensorPosition();

    SmartDashboard.putNumber("Pot Position", cargoArm.getSelectedSensorPosition());

    cargoArm.set(ControlMode.PercentOutput, motorOutput);
  }

  public double deadzone(double val) {
    return Math.abs(val) > 0.05 ? val : 0;
  }

  private double getMaxPower(double potVal, int colRef, double[][] powerTable) {
    double maxOutput = 0;
    
    for(int r = 0; r < powerTable.length; r++) {
        try {
          if(inRangeInclusive(potVal, powerTable[r][0], powerTable[r+1][0])) {
            maxOutput = powerTable[r][colRef];
            return maxOutput;
          }
        } catch(ArrayIndexOutOfBoundsException e) {
          maxOutput = powerTable[r][colRef];
        }
      }
    return maxOutput;
  }

  private boolean inRangeInclusive(double val, double min, double max) {
    return val >= min && val <= max;
  }

  private double getPIDError() {
    return getSetpoint() - getPosition();
  }
}
