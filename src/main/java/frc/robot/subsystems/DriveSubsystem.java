// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.*;

//import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;



public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules

  private final RotateAutoAlign autoAlign;
  private final SwerveDrivePoseEstimator m_PoseEstimator;
  //public final ShooterIntake m_ShooterIntake;
  NetworkTable table;
  boolean canLineUp = false;

  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);
  
  public static final SwerveDriveKinematics kDriveKinematics =
    new SwerveDriveKinematics(
      new Translation2d(DriveConstants.kWheelBase / 2.0, DriveConstants.kTrackWidth / 2.0), // Front Left
      new Translation2d(DriveConstants.kWheelBase / 2.0, -DriveConstants.kTrackWidth / 2.0), // Front Right
      new Translation2d(-DriveConstants.kWheelBase / 2.0, DriveConstants.kTrackWidth / 2.0), // Back Left
      new Translation2d(-DriveConstants.kWheelBase / 2.0, -DriveConstants.kTrackWidth / 2.0) // Back Right
    );
      
  private final Pigeon2 pigeon = new Pigeon2(5, "rio");

  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      pigeon.getRotation2d(),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });

  

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    LimelightHelpers.setPipelineIndex("limelight", 1);
    pigeon.getAccelerationX();
    autoAlign = new RotateAutoAlign();
    table = NetworkTableInstance.getDefault().getTable("limelight-center");

  m_PoseEstimator = new SwerveDrivePoseEstimator(
    kDriveKinematics,
    pigeon.getRotation2d(),
    getModulePositions(),
    new Pose2d(),
    // 1. State standard deviations (X, Y, theta). Lower numbers = trust wheels MORE.
    VecBuilder.fill(0.1, 0.1, 0.1), 
    // 2. Vision standard deviations (X, Y, theta). Lower numbers = trust Limelight MORE.
    VecBuilder.fill(0.9, 0.9, 0.9)  
  );
	  PPHolonomicDriveController driveController = new PPHolonomicDriveController(new PIDConstants(8.0, 0.0, 0.0), new PIDConstants(10, 0.0, 0.0));
    RobotConfig config;

    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      e.printStackTrace();
      throw new RuntimeException("BAD SETTINGS FILE");
    }

    AutoBuilder.configure(
      this::getPose, 
      this::resetOdometry, 
      this::getSpeeds, 
      (speeds, feedforwards) -> driveRobotRelative(speeds), 
      driveController, 
      config,
      () -> {
        return false;
      },
      this
    );

	  HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);
    resetEncoders();
    
  }

  @Override
  public void periodic() {

  m_PoseEstimator.update(pigeon.getRotation2d(), getModulePositions());
  LimelightHelpers.PoseEstimate mt1Result = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-center");
  if (mt1Result.tagCount > 0) {
        // mt1Result.timestampSeconds automatically corrects for the camera's processing lag
        m_PoseEstimator.addVisionMeasurement(mt1Result.pose, mt1Result.timestampSeconds);
    }

  SmartDashboard.putNumber("Yaw", pigeon.getYaw().getValueAsDouble());
	SmartDashboard.putNumber("Pitch", pigeon.getPitch().getValueAsDouble());
	SmartDashboard.putNumber("Roll", pigeon.getRoll().getValueAsDouble());
    
    m_odometry.update(
        pigeon.getRotation2d(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });
  }


  /*public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }*/

  public Pose2d getPose() {
    return m_PoseEstimator.getEstimatedPosition();
}

  public void resetOdometry(Pose2d pose) {
    System.out.println("Odometry Reset");
    pigeon.setYaw(pose.getRotation().getDegrees()); 
    m_odometry.resetPosition(
        pose.getRotation(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // Convert the commanded speeds into the correct units for the drivetrain
	XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  double rightTriggerValue = (m_driverController.getRightTriggerAxis() * -0.8) + 1.0;

  double newRot = rot;
  NetworkTableEntry numberEntry = table.getEntry("tid");
  
  //Auto Align With April Tag
  if(m_driverController.getLeftTriggerAxis() > 0 || m_driverController.getLeftBumperButton() || canLineUp){
    if((double)numberEntry.getDouble(0.0) == 10 ||(double)numberEntry.getDouble(0.0) == 9 || (double)numberEntry.getDouble(0.0) == 8){
      newRot = autoAlign.getRotationSpeed();
    }else{
      newRot = rot;
    }
  }else if(m_driverController.getLeftTriggerAxis() == 0 && !m_driverController.getLeftBumperButton() && !canLineUp){
    newRot = rot;
  }
  
	if(xSpeed < -0.01 && xSpeed > -0.1) {
		xSpeed = 0.0;
	}
	if(ySpeed < 0.01 && ySpeed > 0.01) {
		ySpeed = 0.0;
	}

	xSpeed = xSpeed * rightTriggerValue;
	ySpeed = ySpeed * rightTriggerValue;
	newRot = newRot * rightTriggerValue;

  SmartDashboard.putNumber("rot", (double)newRot);

	double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond ;
    double rotDelivered = newRot * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                getPose().getRotation())
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }


  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  public void zeroHeading() {
    System.out.println("ZERO");
    pigeon.reset();
    resetEncoders();
  }

  public double getHeading() {
    return pigeon.getRotation2d().getDegrees();
  } 

  public double getTurnRate() {
    return pigeon.getAngularVelocityYWorld().getValueAsDouble();
  }

  public ChassisSpeeds getSpeeds() {
    return kDriveKinematics.toChassisSpeeds(new SwerveModuleState[] {m_frontLeft.getState(), m_frontRight.getState(), m_rearLeft.getState(), m_rearRight.getState()});
  }

  public void CanLineUp(boolean lineUp){
    canLineUp = lineUp;
  }

  void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds){
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

    PIDController rotationPID = new PIDController(5,  0.0, 0.0);
    rotationPID.setTolerance(0.05);
    rotationPID.setSetpoint(0.0);
    double tx = LimelightHelpers.getTX("limelight-center");

    if(canLineUp && (tx > 2 || tx < -2)){
      double output = rotationPID.calculate(tx);
      System.out.println("Auto Aligning Rotating: " + output);
      targetSpeeds.omegaRadiansPerSecond = Math.toRadians(output);
    }
    var targetStates = kDriveKinematics.toSwerveModuleStates(targetSpeeds);
    setModuleStates(targetStates);
  }

  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
        m_frontLeft.getPosition(),  // Element 0: Front Left
        m_frontRight.getPosition(), // Element 1: Front Right
        m_rearLeft.getPosition(),   // Element 2: Back Left
        m_rearRight.getPosition()   // Element 3: Back Right
    };
}
}
