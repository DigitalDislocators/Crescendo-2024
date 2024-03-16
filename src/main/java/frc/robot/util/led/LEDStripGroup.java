package frc.robot.util.led;

import org.ejml.simple.UnsupportedOperation;

import edu.wpi.first.wpilibj.util.Color;

public class LEDStripGroup extends LEDStrip {

    private final LEDStrip[] components;

    private final int length;

    public LEDStripGroup(boolean isReversed, LEDStrip... components) {
        super(0, isReversed);
        this.components = components;
        int length = 0;
        for(LEDStrip ledStrip : components) {
            length += ledStrip.getLength();
        }
        this.length = length;
    }

    public LEDStripGroup(LEDStrip... components) {
        this(false, components);
    }

    public LEDStrip[] getComponents() {
        return components;
    }

    public int getLength() {
        return length;
    }

    @Override
    public double getBrightness() {
        throw new UnsupportedOperation("LEDStripGroup has no value for brightness. Get brightness of components instead.");
    }
    @Override
    public void setBrightness(double brightness) {
        for(LEDStrip ledStrip : components) {
            ledStrip.setBrightness(brightness);
        }
    }

    @Override
    public Color[] getPixelBuffer() {
        Color[] pixelBuffer = new Color[length];
        int groupPixel = 0;
        if(isReversed()) {
            for(LEDStrip ledStrip : components) {
                for(int i = ledStrip.getLength() - 1; i >= 0; i--) {
                    pixelBuffer[groupPixel] = ledStrip.getPixel(i);
                    groupPixel++;
                }
            }
        }
        else {
            for(LEDStrip ledStrip : components) {
                for(int i = 0; i < length; i++) {
                    pixelBuffer[groupPixel] = ledStrip.getPixel(i);
                    groupPixel++;
                }
            }
        }
        return pixelBuffer;
    }

    @Override
    public Color getPixel(int index) {
        if(isReversed()) {
            index = length - 1 - index;
        }
        int pixelCount = 0;
        for(LEDStrip ledStrip : components) {
            pixelCount += ledStrip.getLength();
            if(pixelCount > index) {
                return ledStrip.getPixel(index + ledStrip.getLength() - pixelCount);
            }
        }
        throw new ArrayIndexOutOfBoundsException(index);
    }

    @Override
    public Color[] getColorBuffer() {
        throw new UnsupportedOperation("LEDStripGroup has no color buffer. Get color buffer of components instead.");
    }

    @Override
    public Color getColor(int index) {
        throw new UnsupportedOperation("LEDStripGroup has no color buffer. Get color buffer of components instead.");
    }

    @Override
    public double[] getValueBuffer() {
        throw new UnsupportedOperation("LEDStripGroup has no value buffer. Get value buffer of components instead.");
    }

    @Override
    public double getValue(int index) {
        throw new UnsupportedOperation("LEDStripGroup has no value buffer. Get value buffer of components instead.");
    }

    @Override
    public void setColor(Color color) {
        for(LEDStrip ledStrip : components) {
            ledStrip.setColor(color);
        }
    }

    @Override
    public void setColor(Color color, int index) {
        throw new UnsupportedOperation("LEDStripGroup has no indeces. Set color at an index of components instead.");
    }

    @Override
    public void translateColors(TranslateDirection direction, Color... voidColors) {
        for(LEDStrip ledStrip : components) {
            ledStrip.translateColors(direction, voidColors);
        }
    }

    @Override
    public void setValue(double value) {
        for(LEDStrip ledStrip : components) {
            ledStrip.setValue(value);
        }
    }

    @Override
    public void setValue(double value, int index) {
        throw new UnsupportedOperation("LEDStripGroup has no indeces. Set value at an index of components instead.");
    }

    @Override
    public void translateValues(TranslateDirection direction, double... voidValues) {
        for(LEDStrip ledStrip : components) {
            ledStrip.translateValues(direction, voidValues);
        }
    }
    
}
