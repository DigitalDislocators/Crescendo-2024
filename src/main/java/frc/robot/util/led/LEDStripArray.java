package frc.robot.util.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.util.led.LEDParent.TranslateDirection;

public class LEDStripArray {
    
    private final LEDParent[] ledStrips;

    private final AddressableLED driver;

    private final AddressableLEDBuffer buffer;

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

        CommandScheduler.getInstance().schedule(Commands.run(() -> update()));
    }

    public void update() {
        int bufferIndex = 0;
        for(LEDParent ledStrip : ledStrips) {
            for(Color pixel : ledStrip.getPixelBuffer()) {
                buffer.setLED(bufferIndex, pixel);
                bufferIndex++;
            }
        }

        driver.setData(buffer);
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
