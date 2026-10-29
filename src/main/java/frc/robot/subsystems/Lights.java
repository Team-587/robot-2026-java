// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;

public class Lights extends SubsystemBase {
  /** Creates a new Lights. */
  AddressableLED m_LED;
  AddressableLEDBuffer m_buffer;
  int disableLoopCount;
  boolean blueActive;
  double offset;
  private final RotateAutoAlign autoAlign;
  public final ShooterIntake shooter;
  private final CommandXboxController m_driverController =
      new CommandXboxController(OIConstants.kDriverControllerPort);

  public Lights() {
    offset = 0;
    System.out.println("LIGHTS");
    autoAlign = new RotateAutoAlign();
    shooter = new ShooterIntake();
    m_LED = new AddressableLED(OIConstants.kLEDPort);
    m_buffer = new AddressableLEDBuffer(OIConstants.kLEDLength);

    for (int i = 0; i < OIConstants.kLEDLength - 1; i = i + 2){
      m_buffer.setRGB(i, 0, 133, 202);
      m_buffer.setRGB((i + 1), 252, 186, 3);
    }
    System.out.println(m_buffer.getBlue(0));
    m_LED.setLength(OIConstants.kLEDLength);
    m_LED.setData(m_buffer);
    m_LED.start();
    System.out.println("LIGHTS ONNNN");
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Match Time", getMatchTime());
    // This method will be called once per scheduler run
    /*if(isActive(DriverStation.getAlliance().get()) && getCurrentShift().get() != Shift.ENDGAME){
        if(autoAlign.getRotationSpeed() == 0.0){
          for (int i = 0; i < OIConstants.kLEDLength - 1; i++){
            m_buffer.setRGB(i, 52, 235, 85);
          }
        }else{
          for (int i = 0; i < OIConstants.kLEDLength - 1; i++){
            m_buffer.setRGB(i, 0, 133, 202);
          }
        }
    }*/if((DriverStation.isTeleopEnabled() || DriverStation.isAutonomousEnabled())){
        if(shooter.toggle){
          for (int i = 0; i < OIConstants.kLEDLength; i++){
              m_buffer.setRGB(i, 255, 0,255);
            }
        }else{
          if(((autoAlign.getRotationSpeed() < 0.1) && (autoAlign.getRotationSpeed() > -0.1)) && (m_driverController.leftBumper().getAsBoolean() || m_driverController.leftTrigger().getAsBoolean())){
            if(shooter.getCurrentShooterVelocity() < (shooter.getSetShooterSpeed() + 1200) && shooter.getCurrentShooterVelocity() > (shooter.getSetShooterSpeed() + 1000) && shooter.getSetShooterSpeed() != 0.0){
              for (int i = 0; i < OIConstants.kLEDLength; i++){
                m_buffer.setRGB(i, 0, 255,0);
              }
            }else{
              for (int i = 0; i < OIConstants.kLEDLength; i++){
                m_buffer.setRGB(i, 255, 0, 0);
              }
            }
        }else if(!((autoAlign.getRotationSpeed() < 0.1) && (autoAlign.getRotationSpeed() > -0.1)) && (m_driverController.leftBumper().getAsBoolean() || m_driverController.leftTrigger().getAsBoolean())){
          for (int i = 0; i < OIConstants.kLEDLength; i++){
            m_buffer.setRGB(i, 255, 0, 0);
          }
        }else if(!m_driverController.leftBumper().getAsBoolean() && !m_driverController.leftTrigger().getAsBoolean()){
          for (int i = 0; i < OIConstants.kLEDLength; i++){
            m_buffer.setRGB(i, 252, 186, 3);
          }
        }
      }
    }else if(!(DriverStation.isTeleopEnabled() || DriverStation.isAutonomousEnabled())){
      for(int i = 0; i < m_buffer.getLength(); i++){
        double position = (i + offset) / m_buffer.getLength();

        double wave = (Math.sin(position * 2 * Math.PI) + 1) / 2;
        wave = Math.max(0, Math.min(1, wave));

        int r1 = 0, g1 = 133, b1 = 202;
        int r2 = 252, g2 = 186, b2 = 3;

        int r = (int)(r1  + (r2-r1) * (1-wave));
        int g = (int)(g1 + (g2-g1) * (1-wave));
        int b = (int)(b1 + (b2-b1) * (1-wave));

        r = Math.max(0, Math.min(255, r));
        g = Math.max(0, Math.min(255, g));
        b = Math.max(0, Math.min(255, b));

        m_buffer.setRGB(i, r, g, b);
      }
      offset += 0.5;
      offset %= m_buffer.getLength();
    }
    SmartDashboard.putNumber("Set Speed", shooter.getSetShooterSpeed());
    SmartDashboard.putNumber("Get Speed", shooter.getCurrentShooterVelocity());
    m_LED.setData(m_buffer);
  }

  public static Optional<Alliance> getAutoWinner(){
    String msg = DriverStation.getGameSpecificMessage();
    char msgChar = msg.length() > 0 ? msg.charAt(0) : ' ';
    switch (msgChar) {
      case 'B':
        return Optional.of(Alliance.Blue);
      case 'R':
        return Optional.of(Alliance.Red);
      default:
        return Optional.empty();
    }
  }

  public static Optional<Shift> getCurrentShift(){
    double matchTime = getMatchTime();
    if (matchTime < 0) return Optional.empty();

    for(Shift shift : Shift.values()){
      if(matchTime < shift.endTime){
        return Optional.of(shift);
      }
    }
    return Optional.empty();
  }

  public static boolean isActive(Alliance alliance, Shift shift){
    Optional<Alliance> autoWinner = getAutoWinner();
    switch(shift.activeType) {
      case BOTH:
        return true;
      case AUTO_WINNER:
        return autoWinner.isPresent() && autoWinner.get() == alliance;
      case AUTO_LOOSER:
        return autoWinner.isPresent() && autoWinner.get() != alliance;
      default:
        return false;
    }
  }

  public static boolean isActive(Alliance alliance){
    Optional<Shift> currentShift = getCurrentShift();
    return currentShift.isPresent() && isActive(alliance, currentShift.get());
  }

  public static double getMatchTime(){
    if(DriverStation.isAutonomous()){
      if(DriverStation.getMatchTime() < 0) return DriverStation.getMatchTime();
      return DriverStation.getMatchTime();
    }else if(DriverStation.isTeleop()){
      if(DriverStation.getMatchTime() < 0) return DriverStation.getMatchTime();
      return DriverStation.getMatchTime();
    }
    return -1;
  }

  public enum Shift{
    AUTO(0, 20, ActiveType.BOTH),
    TRANSITION(20, 30, ActiveType.BOTH),
    SHIFT_1(30, 55, ActiveType.AUTO_LOOSER),
    SHIFT_2(55, 80, ActiveType.AUTO_WINNER),
    SHIFT_3(80, 105, ActiveType.AUTO_LOOSER),
    SHIFT_4(105, 130, ActiveType.AUTO_WINNER),
    ENDGAME(130, 160, ActiveType.BOTH);

    final int startTime;
    final int endTime;
    final ActiveType activeType;

    private Shift(int startTime, int endTime, ActiveType activeType){
      this.startTime = startTime;
      this.endTime = endTime;
      this.activeType = activeType;
    }
  }

  private enum ActiveType{
    BOTH,
    AUTO_WINNER,
    AUTO_LOOSER
  }
}
