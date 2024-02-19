package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FeederSys extends SubsystemBase {

    Servo leftServo;
    Servo rightServo;

    public FeederSys() {
        leftServo = new Servo(7);
        rightServo = new Servo(8);
    }

    public void setLeftPower(double leftpower) {
        leftServo.set(leftpower);
    }
    public void setRightPower(double rightpower) {
        rightServo.set(rightpower);
    }
}