// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase {

  private double x;
  private double y;
  private double area;
  private boolean v;
  private double distance;

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");   // Horizontal Offset From Crosshair To Target  (-27 degrees to 27 degrees)
    NetworkTableEntry ty = table.getEntry("ty");   // Vertical Offset From Crosshair To Target  (-20.5 degrees to 20.5 degrees)
    NetworkTableEntry ta = table.getEntry("ta");   // Target Area (0% of image to 100% of image)
    NetworkTableEntry tv = table.getEntry("tv");   // Whether the limelight has any valid targets (0 or 1)

    // read values periodically
    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    area = ta.getDouble(0.0);
    v = tv.getBoolean(false);
  }

  public double getx(){
    return x;
  }

  public double gety(){
    return y;
  }

  public double getarea(){
    return area;
  }

  public double getDistance(){
    distance = (VisionConstants.targetHeightMeter - VisionConstants.limelightHeightMeter) / Math.tan(Math.toRadians(VisionConstants.limelightAngle + y));
    return distance;
  }
}
