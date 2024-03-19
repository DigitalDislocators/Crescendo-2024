package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWM;
// import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SpacebarSys extends SubsystemBase {

    PWM leftServo;
    PWM rightServo;

    public SpacebarSys() {
        leftServo = new PWM(5);
        rightServo = new PWM(6);
    }

    public void setPosition(double position) {
        leftServo.setPosition(position);
        rightServo.setPosition(position);
    }
}