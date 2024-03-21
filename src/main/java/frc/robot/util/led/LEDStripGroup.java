package frc.robot.util.led;

import edu.wpi.first.wpilibj.util.Color;

public class LEDStripGroup implements LEDParent {

    private final LEDParent[] components;

    private final Color[] colorBuffer;

    private final double[] valueBuffer;

    private final int length;

    private boolean isReversed;

    public LEDStripGroup(boolean isReversed, LEDParent... components) {
        this.isReversed = isReversed;
        this.components = components;
        int length = 0;
        for(LEDParent ledStrip : components) {
            length += ledStrip.getLength();
        }

        colorBuffer = new Color[length];
        valueBuffer = new double[length];

        for(int i = 0; i < length; i++) {
            colorBuffer[i] = new Color();
            valueBuffer[i] = 1.0;
        }

        this.length = length;
    }

    public LEDStripGroup(LEDStrip... components) {
        this(false, components);
    }

    public LEDParent[] getComponents() {
        return components;
    }

    public int getLength() {
        return length;
    }

    public boolean isReversed() {
        return isReversed;
    }
    public void setReversed(boolean isReversed) {
        this.isReversed = isReversed;
    }

    public void setBrightness(double brightness) {
        for(LEDParent ledStrip : components) {
            ledStrip.setBrightness(brightness);
        }
    }

    public Color[] getLEDBuffer() {
        Color[] pixelBuffer = new Color[length];
        int groupPixel = 0;
        if(isReversed) {
            for(LEDParent ledStrip : components) {
                for(int i = ledStrip.getLength() - 1; i >= 0; i--) {
                    pixelBuffer[groupPixel] = ledStrip.getPixel(i);
                    groupPixel++;
                }
            }
        }
        else {
            for(LEDParent ledStrip : components) {
                for(int i = 0; i < length; i++) {
                    pixelBuffer[groupPixel] = ledStrip.getPixel(i);
                    groupPixel++;
                }
            }
        }
        return pixelBuffer;
    }

    public Color getLED(int index) {
        if(isReversed) {
            index = length - 1 - index;
        }
        int pixelCount = 0;
        for(LEDParent ledStrip : components) {
            pixelCount += ledStrip.getLength();
            if(pixelCount > index) {
                return ledStrip.getPixel(index + ledStrip.getLength() - pixelCount);
            }
        }
        throw new ArrayIndexOutOfBoundsException(index);
    }

    public Color[] getPixelBuffer() {
        Color[] pixelBuffer = new Color[length];
        if(isReversed) {
            for(int i = length - 1; i >= 0; i--) {
                pixelBuffer[i] =
                    new Color(
                        colorBuffer[i].red * valueBuffer[i],
                        colorBuffer[i].green * valueBuffer[i],
                        colorBuffer[i].blue * valueBuffer[i]
                    );
            }
        }
        else {
            for(int i = 0; i < length; i++) {
                pixelBuffer[i] =
                    new Color(
                        colorBuffer[i].red * valueBuffer[i],
                        colorBuffer[i].green * valueBuffer[i],
                        colorBuffer[i].blue * valueBuffer[i]
                    );
            } 
        }
        return pixelBuffer;
    }

    public Color getPixel(int index) {
        if(isReversed) {
            index = length - 1 - index;
        }
        return new Color(
            colorBuffer[index].red * valueBuffer[index],
            colorBuffer[index].green * valueBuffer[index],
            colorBuffer[index].blue * valueBuffer[index]
        );
    }

    public Color[] getColorBuffer() {
        return colorBuffer;
    }
    
    public Color getColor(int index) {
        return colorBuffer[index];
    }

    public double[] getValueBuffer() {
        return valueBuffer;
    }

    public double getValue(int index) {
        return valueBuffer[index];
    }

    public void setColor(Color color) {
        for(int i = 0; i < length; i++) {
            colorBuffer[i] = color;
        }
        
        for(LEDParent ledStrip : components) {
            ledStrip.setColor(color);
        }
    }

    public void setColor(Color color, int index) {
        colorBuffer[index] = color;
        for(LEDParent ledStrip : components) {
            if(index < ledStrip.getLength()) {
                ledStrip.setColor(color, index);
            }
        }
    }

    public void translateColors(TranslateDirection direction, Color... voidColors) {
        if(voidColors.length > length) {
            Color[] proxyVoidColors = new Color[length];
            for(int i = 0; i < length; i++) {
                proxyVoidColors[i] = voidColors[i];
            }
            voidColors = proxyVoidColors;
        }
        if(direction == TranslateDirection.FORWARD) {
            for(int i = voidColors.length; i < length; i++) {
                colorBuffer[i] = colorBuffer[i - voidColors.length];
            }
            for(int i = 0; i < voidColors.length; i++) {
                colorBuffer[i] = voidColors[i];
            }
        }
        else {
            for(int i = length - voidColors.length - 1; i >= 0; i--) {
                colorBuffer[i] = colorBuffer[i + voidColors.length];
            }
            for(int i = 0; i < voidColors.length; i++) {
                colorBuffer[length - voidColors.length + i] = voidColors[i];
            }
        }

        for(LEDParent ledStrip : components) {
            ledStrip.translateColors(direction, voidColors);
        }
    }

    public void setValue(double value) {
        for(int i = 0; i < length; i++) {
            valueBuffer[i] = value;
        }

        for(LEDParent ledStrip : components) {
            ledStrip.setValue(value);
        }
    }

    public void setValue(double value, int index) {
        valueBuffer[index] = value;
        for(LEDParent ledStrip : components) {
            if(index < ledStrip.getLength()) {
                ledStrip.setValue(value, index);
            }
        }
    }

    public void translateValues(TranslateDirection direction, double... voidValues) {
        if(direction == TranslateDirection.FORWARD) {
            for(int i = voidValues.length; i < length; i++) {
                valueBuffer[i] = valueBuffer[i - voidValues.length];
            }
            for(int i = 0; i < voidValues.length; i++) {
                valueBuffer[i] = valueBuffer[i];
            }
        }
        else {
            for(int i = length - voidValues.length - 1; i >= 0; i--) {
                valueBuffer[i] = valueBuffer[i + voidValues.length];
            }
            for(int i = 0; i < voidValues.length; i++) {
                valueBuffer[length - voidValues.length + i] = voidValues[i];
            }
        }

        for(LEDParent ledStrip : components) {
            ledStrip.translateValues(direction, voidValues);
        }
    }
}
