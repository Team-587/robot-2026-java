// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class RotateAutoAlign extends SubsystemBase {

  PIDController rotationPID;
  NetworkTable table;

  double cameraHeight;
  double tagHeight;
  double cameraAngle;
  double lastTY;

  ArrayList<Double> tys;

  private final Field2d m_field = new Field2d();

  /** Creates a new RotateAutoAlign. */
  public RotateAutoAlign() {
    LimelightHelpers.setPipelineIndex("limelight", 1);
    rotationPID = new PIDController(0.025, 0.0, 0.0);
    table = NetworkTableInstance.getDefault().getTable("limelight-center");
    rotationPID.setTolerance(0.05);
    rotationPID.setSetpoint(0);
    cameraHeight = 16.5;
    tagHeight = 44.3;
    cameraAngle = 14;

    tys = new ArrayList<Double>();

    tys.add(0.0);
    tys.add(0.0);
    tys.add(0.0);
    tys.add(0.0);
    tys.add(0.0);

    SmartDashboard.putData("Field", m_field);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    LimelightHelpers.PoseEstimate mt1Result = 
    LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-center");

    if(mt1Result.tagCount > 0){
      Pose2d robotPose = mt1Result.pose;

      m_field.setRobotPose(robotPose);

      m_field.getObject("MegaTag1_Pose").setPose(robotPose);

      SmartDashboard.putNumber("MT1_X", robotPose.getX());
      SmartDashboard.putNumber("MT1_Y", robotPose.getY());
    }
  }

  public double getDistance(){
    double ty = LimelightHelpers.getTY("limelight-center");

    if(ty != 0){
      lastTY = ty;
    }

    tys.remove(0);
    tys.add(lastTY);

    double total = 0;
    for (double i : tys){
      total += i;
    }

    double tyAverage = total/tys.size();
    double zDistance = (tagHeight-cameraHeight)/ Math.tan(Math.toRadians(cameraAngle + tyAverage));

    return zDistance;
  }
  
  public double getRotationSpeed(){
    double tx = LimelightHelpers.getTX("limelight-center");
    //double[] positions;
    //positions = LimelightHelpers.getBotPose_TargetSpace("limelight-center");
    double output = rotationPID.calculate(tx);

    SmartDashboard.putNumber("TX", tx);
    SmartDashboard.putNumber("output", output);

    return output;
  }
}
