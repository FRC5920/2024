// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.HAL;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.ConfigurationFailedException;

public class HALSubsystem extends SubsystemBase {
    private LaserCan lcHALLeft;
    private LaserCan lcHALRight;
  /** Creates a new HALSubsystem. */
  public HALSubsystem() {
    try {
      lcHALLeft.setRangingMode(LaserCan.RangingMode.LONG);
      lcHALLeft.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
      lcHALLeft.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
    } catch (ConfigurationFailedException e) {
      System.out.println("Configuration failed! " + e);
    }
    try {
      lcHALRight.setRangingMode(LaserCan.RangingMode.LONG);
      lcHALRight.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
      lcHALRight.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
    } catch (ConfigurationFailedException e) {
      System.out.println("Configuration failed! " + e);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    LaserCan.Measurement measurementLeft = lcHALLeft.getMeasurement();
    if (measurementLeft != null && measurementLeft.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      System.out.println("The target is " + measurementLeft.distance_mm + "mm away!");
    } else {
      System.out.println("Oh no! The target is out of range, or we can't get a reliable measurement!");
      // You can still use distance_mm in here, if you're ok tolerating a clamped value or an unreliable measurement.
    }
       LaserCan.Measurement measurementRight = lcHALRight.getMeasurement();
    if (measurementRight != null && measurementRight.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      System.out.println("The target is " + measurementRight.distance_mm + "mm away!");
    } else {
      System.out.println("Oh no! The target is out of range, or we can't get a reliable measurement!");
      // You can still use distance_mm in here, if you're ok tolerating a clamped value or an unreliable measurement.
    }
  }
}
