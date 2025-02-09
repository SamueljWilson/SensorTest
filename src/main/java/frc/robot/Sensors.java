// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import com.playingwithfusion.TimeOfFlight;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Sensors extends SubsystemBase {
  LaserCan m_lc;
  TimeOfFlight m_tof;
  DigitalInput m_pwmSource;
  DutyCycle m_pwm;
  int m_counter = 0;
  double m_startTime = 0.0;

  ArrayList<Integer> m_LCReadings = new ArrayList<Integer>();
  ArrayList<Integer> m_TOFReadings = new ArrayList<Integer>();
  ArrayList<Integer> m_PWMReadings = new ArrayList<Integer>();

  public Sensors() {
    m_lc = new LaserCan(3);
    m_tof = new TimeOfFlight(2);
    m_pwmSource = new DigitalInput(1);
    m_pwm = new DutyCycle(m_pwmSource);

    try {
      m_lc.setRangingMode(LaserCan.RangingMode.SHORT);
      m_lc.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
      m_lc.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
    } catch (ConfigurationFailedException e) {
      SmartDashboard.putString("LC Sensor config", e.toString());
    }

    m_tof.setRangingMode(TimeOfFlight.RangingMode.Short, 33);
    m_tof.setRangeOfInterest(0, 15, 15, 0);

  }

  private String lcStatusToString(int status) {
    switch (status) {
      case LaserCanInterface.LASERCAN_STATUS_VALID_MEASUREMENT:
        return "Valid";
      case LaserCanInterface.LASERCAN_STATUS_NOISE_ISSUE:
        return "Noise Issue";
      case LaserCanInterface.LASERCAN_STATUS_OUT_OF_BOUNDS:
        return "Measurement Out Of Bounds";
      case LaserCanInterface.LASERCAN_STATUS_WEAK_SIGNAL:
        return "Weak Signal";
      case LaserCanInterface.LASERCAN_STATUS_WRAPAROUND:
        return "Wraparound";
      default:
        assert(false);
        return "Unkown Error";
    } 
  }

  public int getLCMeasurement() {
    return m_lc.getMeasurement().distance_mm;
  }

  public int getTOFMeasurement() {
    return (int)m_tof.getRange();
  }

  public int getPWMMeasurement() {
    int pwmTime = m_pwm.getHighTimeNanoseconds() / 1000;
    int pwmDistance;
    if (pwmTime == 0) {
      pwmDistance = -1;
    } else if (pwmTime > 1850) {
      pwmDistance = -1;
    } else {
      pwmDistance = Math.max(0, 4 * (pwmTime - 1000));
    }
    return pwmDistance;
  }

  @Override
  public void periodic() {
    LaserCan.Measurement lcMeasurement = m_lc.getMeasurement();
    if (lcMeasurement != null) {
      SmartDashboard.putNumber("LC Distance", lcMeasurement.distance_mm);
      SmartDashboard.putString("LC Status", lcStatusToString(lcMeasurement.status));
    } else {
      SmartDashboard.putNumber("LC Distance", -1);
    }

    double tofDistance = m_tof.getRange();
    SmartDashboard.putNumber("TOF Distance", tofDistance);
    SmartDashboard.putString("TOF Status", m_tof.getStatus().toString());

    int pwmTime = m_pwm.getHighTimeNanoseconds() / 1000;
    int pwmDistance;
    SmartDashboard.putNumber("PWM Raw Time", pwmTime);
    if (pwmTime == 0) {
      pwmDistance = 500;
      SmartDashboard.putString("PWM Status", "timeout");
    } else if (pwmTime > 1850) {
      pwmDistance = 500;
      SmartDashboard.putNumber("PWM Distance", pwmDistance);
      SmartDashboard.putString("PWM Status", "Invalid Distance");
    } else {
      pwmDistance = Math.max(0, 4 * (pwmTime - 1000));
      SmartDashboard.putNumber("PWM Distance", pwmDistance);
      SmartDashboard.putString("PWM Status", "Valid Distance");
    }

    SmartDashboard.putNumber("LC and TOF Difference", Math.abs(lcMeasurement.distance_mm - tofDistance));
    SmartDashboard.putNumber("LC and PWM Difference", Math.abs(lcMeasurement.distance_mm - pwmDistance));
    SmartDashboard.putNumber("TOF and PWM Difference", Math.abs(tofDistance - pwmDistance));

    if (DriverStation.isTeleopEnabled()) {
      double currentTime = (double)RobotController.getFPGATime() / 1000000.0;
      if (m_startTime == 0.0) {
        m_startTime = currentTime;
      } else if (m_counter > 4) {
        System.out.println("LC Readings:" + m_LCReadings.toString());
        System.out.println("TOF Readings:" + m_TOFReadings.toString());
        System.out.println("PWM Readings:" + m_PWMReadings.toString());
        m_counter = 0;
        assert(false);
      }
      else if (currentTime - m_startTime >= 1.0) {
        m_LCReadings.add(getLCMeasurement());
        m_TOFReadings.add(getTOFMeasurement());
        m_PWMReadings.add(getPWMMeasurement());
        m_counter++;
        m_startTime = 0.0;
      }
    }
  }
}
