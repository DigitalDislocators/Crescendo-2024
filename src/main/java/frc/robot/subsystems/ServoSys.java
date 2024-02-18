package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;



public class ServoSys {
    public ServoSys () {
        var leftServo = new Servo(7);
        var rightServo = new Servo(8);

leftServo.set(0.2);
rightServo.set(0.2);
    }
}
