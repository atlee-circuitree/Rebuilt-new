package frc.robot.util;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.hal.PWMConfigDataResult;
import edu.wpi.first.math.MathUtil;

public class HitecServo extends Servo {
    double curPos;

    public HitecServo(int channel) {
        super(0);
        setBoundsMicroseconds(1950, 0, 0, 0, 1050);
        curPos = 0;
        setPosition(curPos);
    }

    public void setPosition(double setpoint) {
        curPos = setpoint;
        if (curPos < 0.0)
            curPos = 0.0;
        if (curPos > 1.0)
            curPos = 1.0;
        super.setPosition(curPos);
    }

    public double getCurPosition() {
        return curPos;
    }

    public void periodic() {
        SmartDashboard.putNumber("Servo pos", curPos);
    }
}