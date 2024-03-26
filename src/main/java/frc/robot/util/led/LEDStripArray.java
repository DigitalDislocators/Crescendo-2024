package frc.robot.util.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.led.LEDParent.TranslateDirection;

public class LEDStripArray extends SubsystemBase{
    
    private final LEDParent[] ledStrips;

    private final AddressableLED driver;

    private final AddressableLEDBuffer buffer;

    private boolean isPaused = false;

    public LEDStripArray(int pwmPort, LEDParent... ledStrips) {
        this.ledStrips = ledStrips;

        driver = new AddressableLED(pwmPort);
        
        int length = 0;
        for(LEDParent ledStrip : ledStrips) {
            length += ledStrip.getLength();
        }

        buffer = new AddressableLEDBuffer(length);

        driver.setLength(length);

        driver.setData(buffer);

        driver.start();
    }

    @Override
    public void periodic() {
        if(isPaused) return;
        
        int bufferIndex = 0;
        for(LEDParent ledStrip : ledStrips) {
            for(Color led : ledStrip.getLEDBuffer()) {
                buffer.setLED(bufferIndex, led);
                bufferIndex++;
            }
        }

        driver.setData(buffer);
    }

    public boolean isPaused() {
        return isPaused;
    }
    public void setPaused(boolean isPaused) {
        this.isPaused = isPaused;
    }

    public void setColors(Color color) {
        for(LEDParent ledStrip : ledStrips) {
            ledStrip.setColor(color);
        }
    }

    public void translateColors(TranslateDirection direction, Color... colors) {
        for(LEDParent ledStrip : ledStrips) {
            ledStrip.translateColors(direction, colors);
        }
    }

    public void setValues(double value) {
        for(LEDParent ledStrip : ledStrips) {
            ledStrip.setValue(value);
        }
    }

    public void translateValues(TranslateDirection direction, double... values) {
        for(LEDParent ledStrip : ledStrips) {
            ledStrip.translateValues(direction, values);
        }
    }

    public void setBrightnesses(double brightness) {
        for(LEDParent ledStrip : ledStrips) {
            ledStrip.setBrightness(brightness);
        }
    }
}
