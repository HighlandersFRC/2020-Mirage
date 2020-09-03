/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.SPI.Port;
import frc.robot.sensors.DriveEncoder;
import frc.robot.sensors.LidarLite;
import frc.robot.sensors.Navx;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;
import frc.robot.tools.pathTools.PathList;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

  public static AHRS navx = new AHRS(Port.kMXP);

  public static Navx mainNavx = new Navx(navx);
    
  public static Relay visionRelay1 = new Relay(0);
  public static Spark ledPort = new Spark(0);

  public static Counter lidarCounter = new Counter(0);
  public static LidarLite lidarLite = new LidarLite(lidarCounter);
	public static int rightMasterTalonID = 6;
  public static int leftMasterTalonID = 1;

  public static int rightFollowerTalon1ID = 5;
  public static int leftFollowerTalon1ID = 2;

  public static int rightFollowerTalon2ID = 4;
  public static int leftFollowerTalon2ID = 3;

  public static int shooterMasterID = 7;
  public static int shooterFollowerID = 8;




  public static TalonSRX leftDriveLead = new TalonSRX(leftMasterTalonID);
  public static TalonSRX rightDriveLead = new TalonSRX(rightMasterTalonID);

	public static TalonSRX leftDriveFollowerOne = new TalonSRX(leftFollowerTalon1ID);
  public static TalonSRX rightDriveFollowerOne = new TalonSRX(rightFollowerTalon1ID);

  public static TalonSRX leftDriveFollowerTwo = new TalonSRX(leftFollowerTalon2ID);
  public static TalonSRX rightDriveFollowerTwo = new TalonSRX(rightFollowerTalon2ID);
  
  public static CANSparkMax shooterMotorOne = new CANSparkMax(shooterMasterID, MotorType.kBrushless);
  public static CANSparkMax shooterMotorTwo = new CANSparkMax(shooterFollowerID, MotorType.kBrushless);


  public static PathList pathlist = new PathList();

  public static TalonSRX driveMotors[] = {
    RobotMap.leftDriveLead,
    RobotMap.rightDriveLead,
    RobotMap.leftDriveFollowerOne,
    RobotMap.rightDriveFollowerOne,
    RobotMap.leftDriveFollowerTwo,
    RobotMap.rightDriveFollowerTwo
  };
  public static TalonSRX driveMotorLeads[] = {
    RobotMap.leftDriveLead,
    RobotMap.rightDriveLead,
  };
  public static TalonSRX allMotorLeads[] = {
    RobotMap.leftDriveLead,
    RobotMap.rightDriveLead,
  };
  public static TalonSRX allMotors[] = {
    RobotMap.leftDriveLead,
    RobotMap.rightDriveLead,
    RobotMap.leftDriveFollowerOne,
    RobotMap.rightDriveFollowerOne,
    RobotMap.leftDriveFollowerTwo,
    RobotMap.rightDriveFollowerTwo
  };

  public static DriveTrain drive = new DriveTrain();
  public static Shooter shooter = new Shooter();
  
}
