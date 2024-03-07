package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWM;
// import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FeederSys extends SubsystemBase {

    PWM leftServo;
    PWM rightServo;

    public FeederSys() {
        leftServo = new PWM(7);
        rightServo = new PWM(8);
    }

    // public void setLeftPower(double leftpower) {
    //     leftServo.set(leftpower);
    // }
    // public void setRightPower(double rightpower) {
    //     rightServo.set(rightpower);
    // }

    public void setPower(double power) {
        leftServo.setSpeed(-power);
        rightServo.setSpeed(power);
    }
}