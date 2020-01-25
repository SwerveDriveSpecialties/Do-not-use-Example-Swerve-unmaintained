/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.subsystems;

import org.opencv.core.Mat;
import org.opencv.osgi.OpenCVInterface;
import org.opencv.video.Video;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.VideoSink;
import edu.wpi.cscore.VideoSource;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  private NetworkTable nt = NetworkTableInstance.getDefault().getTable("limelight");
  private NetworkTableEntry tx = nt.getEntry("tx");
  private NetworkTableEntry ta = nt.getEntry("ta");
  private NetworkTableEntry tv = nt.getEntry("tv");
  private NetworkTableEntry tshort = nt.getEntry("tshort");
  private NetworkTableEntry ty = nt.getEntry("ty");
  private NetworkTableEntry ts = nt.getEntry("ts");
  private NetworkTableEntry pipeline = nt.getEntry("pipeline");

  private double distance;

  public enum Target {
    POWERCELL, HIGH, LOW, OLD;
  }

  private static Limelight _instance = new Limelight();

  public static Limelight getInstance() {
    return _instance;
  }

  private Limelight() {
  }

  public double getAngle1() {
    return tx.getDouble(0);
  }

  public double getTA() {
    return ta.getDouble(0);
  }

  public double getBoxShortLength() {
    return tshort.getDouble(0);
  }

  public double getSkew() {
    return ts.getDouble(0);
  }

  public double getYAng() {
    return ty.getDouble(0);
  }

  public double getDistanceToTarget(Target obj) {
    Target target = obj;
    switch (target) {
    default:
      distance = 0;
    case POWERCELL:
      double hypot = 63.971 * Math.pow(ta.getDouble(0), -0.461);
      distance = hypot > 8.5 ? Math.sqrt(hypot * hypot - 8.5 * 8.5) : 0;
      break;
    case HIGH:
      // distance = 214.81 * Math.pow(ta.getDouble(0), -0.418);
      // distance = Math.sqrt(Math.pow(9448.3 * Math.pow(tshort.getDouble(0), -0.904),
      // 2) - Math.pow(94, 2));
      distance = 86.25 / Math.tan(Math.toRadians(6.42 + ty.getDouble(0)));
      break;
    case LOW:
      distance = 0;
      break;
    }
    ;
    return distance;
  }

  public void setPipeline(double pipe) {
    pipeline.setDouble(pipe);
  }

  public boolean getHasTarget() {
    return tv.getDouble(0.0) != 0.0;
  }
}
