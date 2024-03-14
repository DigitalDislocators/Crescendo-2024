package frc.robot.util.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.util.led.LEDStrip.TranslateDirection;

public class LEDStripArray {
    
    private final LEDStrip[] ledStrips;

    private final AddressableLED driver;

    private final AddressableLEDBuffer buffer;

    public LEDStripArray(int pwmPort, LEDStrip... ledStrips) {
        this.ledStrips = ledStrips;

        driver = new AddressableLED(pwmPort);
        
        int length = 0;
        for(LEDStrip ledStrip : ledStrips) {
            length += ledStrip.getLength();
        }

        buffer = new AddressableLEDBuffer(length);

        driver.setLength(length);

        driver.setData(buffer);
    }

    public void update() {
        int bufferIndex = 0;
        for(LEDStrip ledStrip : ledStrips) {
            for(Color pixel : ledStrip.getPixelBuffer()) {
                buffer.setLED(bufferIndex, pixel);
                bufferIndex++;
            }
        }

        driver.setData(buffer);
    }

    public void setColors(Color color) {
        for(LEDStrip ledStrip : ledStrips) {
            ledStrip.setColor(color);
        }
    }

    public void translateColors(TranslateDirection direction, Color... colors) {
        for(LEDStrip ledStrip : ledStrips) {
            ledStrip.translateColors(direction, colors);
        }
    }

    public void setValues(double value) {
        for(LEDStrip ledStrip : ledStrips) {
            ledStrip.setValue(value);
        }
    }

    public void translateValues(TranslateDirection direction, double... values) {
        for(LEDStrip ledStrip : ledStrips) {
            ledStrip.translateValues(direction, values);
        }
    }

    public void setBrightnesses(double brightness) {
        for(LEDStrip ledStrip : ledStrips) {
            ledStrip.setBrightness(brightness);
        }
    }
}
