/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.sensors;

/**
 * Add your docs here.
 */
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
public class LidarLite {

private static final int distOffset = -18;
private Counter counter;
    public LidarLite (Counter lidarCounter) {
        counter = lidarCounter;
        counter.setMaxPeriod(1.0);
        counter.setSemiPeriodMode(true);
        counter.reset();
    }

    public double getDistance() {
        double cmDist;
        double ftDist;
        if (counter.get() < 1) {
          SmartDashboard.putNumber("lidarDist", -1);
          return -1;
        }
        cmDist = (counter.getPeriod() * 1000000.0 / 10.0) + distOffset;
        ftDist = cmDist*0.0328084;
        SmartDashboard.putNumber("lidarDist", ftDist);
        return ftDist;
    }
}
