package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSys extends SubsystemBase {

    // Declare actuators, sensors, and other variables here

    /**
     * Constructs a new ExampleSys.
     * 
     * <p>ExampleSys contains the basic framework of a robot subsystem for use in command-based programming.
     */
    public LEDSys() {
        // Initialize and configure actuators and sensors here
    LEDStrip = new AddressableLED(0);
    }

    public void Initialize() {

    }

    // This method will be called once per scheduler run
    @Override
    public void periodic() {
        
    }

    // Put methods for controlling this subsystem here. Call these from Commands.

}