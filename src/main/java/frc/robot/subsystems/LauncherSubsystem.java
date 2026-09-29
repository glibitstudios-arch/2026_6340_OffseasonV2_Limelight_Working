// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import tinyspline.BSpline;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import swervelib.simulation.ironmaple.utils.LegacyFieldMirroringUtils2024;

public class LauncherSubsystem extends SubsystemBase {

    TalonFX leftShooter;
    TalonFX rightShooter;
    TalonFX hoodMotor; // Added by michaudc 04 Sep 26

    SparkFlex feederMotor;
    SparkFlex feederMotorRight;
    SparkFlex activeFloorFront;
    SparkFlex activeFloorBack;

    private double reverse = -1;
    private double forward =1;

    private final VelocityVoltage shooter_request = new VelocityVoltage(0).withSlot(0);
    
    // This is for testing - do not use
    PositionVoltage pos_request_Hood = new PositionVoltage(0).withSlot(0); //set motor's pos setpoint to pos specified 7

    // Magic Motion Request - Feedforward setting with Motion Magic
    MotionMagicVoltage m_hood_request = new MotionMagicVoltage(0);

    private double hoodPos = 0;
    

    //private final RelativeEncoder m_leftLaunchEncoder;
    //private final RelativeEncoder m_rightLaunchEncoder;
  /** Creates a new ExampleSubsystem. */
  public LauncherSubsystem() {
    leftShooter = new TalonFX(Constants.LauncherConstants.leftShooterID);
    leftShooter.setNeutralMode(NeutralModeValue.Coast);

    rightShooter = new TalonFX(Constants.LauncherConstants.rightShooterID);
    rightShooter.setNeutralMode(NeutralModeValue.Coast);

    // Added michaudc 04 Sep 26
    hoodMotor = new TalonFX(Constants.LauncherConstants.hoodMotorID);
    hoodMotor.setNeutralMode(NeutralModeValue.Brake);

    // Shooter Motor Setup
    MotorOutputConfigs rightShooterConfigs = new MotorOutputConfigs();
    rightShooterConfigs.Inverted=InvertedValue.CounterClockwise_Positive;
    rightShooter.getConfigurator().apply(rightShooterConfigs);

    MotorOutputConfigs leftShooterConfigs = new MotorOutputConfigs();
    leftShooterConfigs.Inverted=InvertedValue.Clockwise_Positive;
    leftShooter.getConfigurator().apply(leftShooterConfigs);

    leftShooter.setNeutralMode(NeutralModeValue.Coast); // Set to Coast Mode: michaudc
    rightShooter.setNeutralMode(NeutralModeValue.Coast);

    rightShooter.getConfigurator().apply(Constants.LauncherConstants.launcherConfig); // Set to Configs in Contants
    leftShooter.getConfigurator().apply(Constants.LauncherConstants.launcherConfig);

    // Feeder Motors
    feederMotor = new SparkFlex(Constants.LauncherConstants.leftIndexerID, MotorType.kBrushless);
    feederMotorRight = new SparkFlex(Constants.LauncherConstants.rightIndexerID, MotorType.kBrushless);
    activeFloorFront = new SparkFlex(Constants.LauncherConstants.activeFloorFrontID, MotorType.kBrushless);// change from null to something else later
    activeFloorBack = new SparkFlex(Constants.LauncherConstants.activeFloorBackID, MotorType.kBrushless);// change from null to something else later

    


    // SparkMaxConfig launcherConfig = new SparkMaxConfig();
    // launcherConfig.smartCurrentLimit(Constants.LauncherConstants.launcherCurrentLimit);

        // launcherConfig.closedLoop
        // .p(0.00015)
        // .i(0)
        // .d(0)
        // .outputRange(0, 0.95)
        // .feedForward.kV( 12.0 / 6271 ); // 12 Volts divided by Maximum RPM of KrakenX60 (12.0 / 6271)
    
        //Configfor for feeder and active floor
        SparkMaxConfig feederConfig = new SparkMaxConfig();
        feederConfig.smartCurrentLimit(Constants.LauncherConstants.launcherCurrentLimit);
        feederConfig.idleMode(IdleMode.kBrake);


        feederMotor.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        feederMotorRight.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        activeFloorFront.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        activeFloorBack.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Motion Magic Settings for Hood Motor - michaudc 12 Sep 26
        var talonFXConfigs = new TalonFXConfiguration();

        // Slot 0 gains
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = 0; // Static Friction
        slot0Configs.kV = 0.12; // target velocity of 1 rps is 0.12 Volts input
        slot0Configs.kA = 0.01; // acceleration of 1 rps/s requires 0.01 Volts
        slot0Configs.kP = 0.1;  // Constant of Proportion - we will adjust this
        slot0Configs.kI = 0;    // Contsant of Integration
        slot0Configs.kD = 0;    // Constant of Derivative

        // Motion Magic Settings - setting cruise velocity and max speed
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 100; // Max 100 rpm of drive motor
        motionMagicConfigs.MotionMagicAcceleration = 100; // Acceleration of about 5 rps
        motionMagicConfigs.MotionMagicJerk = 500; // Target jerk of 500 rps

        // Apply to Hood Motor
        hoodMotor.getConfigurator().apply(talonFXConfigs);

        //launcherConfig.disableFollowerMode();

      // Invert Left
      //launcherConfig.follow(launcherRight); // Trying to have left follow the right
      //leftShooter.getConfigurator().apply(Constants.LauncherConstants.launcherConfig);

      // Encoders for Launching Motors
       //m_leftLaunchEncoder = leftShooter.getEncoder();
       //m_rightLaunchEncoder =  rightShooter.getEncoder();

    //   m_leftLaunchEncoder.setPosition(0);
    //   m_rightLaunchEncoder.setPosition(0);

      // Smart Dashboard
      SmartDashboard.putNumber("Left Launcher RPM", 0);
      SmartDashboard.putNumber("Right Launcher RPM:", 0);
      SmartDashboard.putNumber("Left Launch Amps", 0);
      SmartDashboard.putNumber("Right Launch Amps", 0);
      double rightShooterVelocity = rightShooter.getVelocity().getValueAsDouble()*60;
      double leftShooterVelocity = leftShooter.getVelocity().getValueAsDouble()*60;

      SmartDashboard.putNumber("Shoot Velocity Right", rightShooterVelocity);
      SmartDashboard.putNumber("Shoot Velocity Left", leftShooterVelocity);
      
      SmartDashboard.putNumber("Hood Position", 0);

  }

  public static double[] getCurvedAngle(double x) {

    double[] points = {
        10, 3000, 20,
        20, 4000, 30,
        30, 5000, 40
    };

    BSpline spline = BSpline.interpolateCubicNatural(points, 3)
            .tension(0.5);


    double u = (x - points[0]) / (points[6] - points[0]);

    double[] p = spline.eval(u).result();

    return new double[] {p[1], p[2]};
  }

  public void setRightShooterVelocity(double velocity){
    rightShooter.setControl(shooter_request.withVelocity(velocity).withFeedForward(0.5));
  }

    public void setLeftShooterVelocity(double velocity){
    leftShooter.setControl(shooter_request.withVelocity(velocity).withFeedForward(0.5));
  }

  public void setShooterVelocity(double velocity){
    //System.out.println("Shooting");
    rightShooter.setControl(shooter_request.withVelocity(-velocity).withFeedForward(0.5));
    leftShooter.setControl(shooter_request.withVelocity(velocity).withFeedForward(0.5));
  }

  public void setFeederSpeed(double power) {
    feederMotor.set(-power);
    feederMotorRight.set(power);
  }



  public void setActiveFloorPower(double power){
    activeFloorBack.set(power);
    activeFloorFront.set(power);  }




    public void setIndexerAndFloorSpeed(double power) {
    feederMotorRight.set(power*2);
    feederMotor.set(reverse*(power*2));
    activeFloorBack.set(power*-1);
    activeFloorFront.set(power*-1);
  }

  //shooter below
    public void setShooterSpeed(double power){
      rightShooter.set(power);
      leftShooter.set(power);
  }

  // This method is for testing only
  public void setHoodSpeed(double velocity) {
    hoodMotor.set(velocity);
  }

    // Magic Motion Position Control - michaudc 12 Sep 26
    public void setHoodPos(double pos) {
      //hoodMotor.setControl(pos_request_Hood.withPosition(pos));
      hoodMotor.setControl(m_hood_request.withPosition(pos));
    }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Updated by michaudc - resuse the key that already exists
    double rightShooterVelocity = rightShooter.getVelocity().getValueAsDouble()*60;
    double leftShooterVelocity = leftShooter.getVelocity().getValueAsDouble()*60;

    SmartDashboard.putNumber("Shoot Velocity Right", rightShooterVelocity);
    SmartDashboard.putNumber("Shoot Velocity Left", leftShooterVelocity);

    //TODO: Put the Positional Value of the Hood Motor
    hoodPos = hoodMotor.getPosition().getValue().magnitude();
    SmartDashboard.putNumber("Hood Position", hoodPos);

  }

  public Command setShooterVelocityCommand(double speed){
    return this.run(()-> setShooterVelocity(speed));
  }

  public Command stopShooterCommand(){
    return this.run(()-> setShooterSpeed(0));
  }

  public Command startFloorCommand(){
    return Commands.run(()-> setActiveFloorPower(.2));
  }

  public Command startStopFloorCommand() {
    return Commands.startEnd(() -> setActiveFloorPower(-0.2), 
    () -> setActiveFloorPower(0));
  }

    public Command stopFloorCommand(){
    return Commands.run(()-> setActiveFloorPower(0));
  }

  public Command feederSpeedCommand(double speed) {
    return Commands.run(() -> setFeederSpeed(speed));
  }

    public Command stopFeederCommand()  {
    return Commands.run(() -> setFeederSpeed(0));
  }


    public Command stopActiveFloorCommand(){
      return Commands.run(()-> setActiveFloorPower(0));
    }

  public Command startFeederCommand(){
    return Commands.run(()-> setFeederSpeed(0.2));
  }
  
    public Command reverseFeederCommand(){
    return Commands.run(()-> setFeederSpeed(-0.2));
  }

  public Command stopIndexerAndFloorCommand()  {
    return Commands.run(() -> setIndexerAndFloorSpeed(0));
  }

    public Command startIndexerAndFloorCommand()  {
    return Commands.run(() -> setIndexerAndFloorSpeed(0.8));
  }
  
    public Command reverseIndexerAndFloorCommand(){
    return Commands.run(()-> setFeederSpeed(-0.8));
  }
    // shooter below
    public Command setShooterSpeedCmd(double speed) {
    return this.startEnd(() -> {
              setShooterSpeed(speed);
       }, () -> {
           setShooterSpeed(0);
       });
    }

    public Command startShooter(){
      return Commands.run(()->setShooterSpeedCmd(0.8));
    }

    // Testing Commands - Do not use in Competition
    public Command hoodDownCommand() {
        return Commands.runOnce(() -> this.setHoodSpeed(-.1));   
    }

    public Command hoodUpCommand() {
      return Commands.runOnce(() -> this.setHoodSpeed(.1));
    }  

    public Command hoodStopCommand() {
      return Commands.runOnce(() -> this.setHoodSpeed(0));
    }
    // End Testing Commands
    
    // Hood Position Command - Use in Competition
    public Command setHoodPositionCommand(double pos) {
      return Commands.runOnce(() -> this.setHoodPos(pos));
    }

}
