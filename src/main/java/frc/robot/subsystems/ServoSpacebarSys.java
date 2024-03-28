package frc.robot.subsystems;


import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ServoSpacebarSys extends SubsystemBase {

    Servo leftServo;
    Servo rightServo;

    public ServoSpacebarSys() {
        leftServo = new Servo(6);
        rightServo = new Servo(5);
    }

    public void setSpacebarLeftPosition(double position) {
        leftServo.setPosition(position);
    }

    public void setSpacebarRightPosition(double position) {
        rightServo.setPosition(position);
    
    }
}