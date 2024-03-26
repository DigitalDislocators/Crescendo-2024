package frc.robot.util.led;

import edu.wpi.first.wpilibj.util.Color;

public class LEDStrip implements LEDParent {

    private final int length;

    private boolean isReversed = false;

    private double brightness = 1.0;

    private Color[] colorBuffer;
    private double[] valueBuffer;

    public LEDStrip(int length, double brightness, boolean isReversed) {
        this.length = length;
        this.brightness = brightness;
        this.isReversed = isReversed;

        colorBuffer = new Color[length];
        valueBuffer = new double[length];

        for(int i = 0; i < length; i++) {
            colorBuffer[i] = new Color();
            valueBuffer[i] = 1.0;
        }
    }

    public LEDStrip(int length) {
        this(length, 1.0, false);
    }

    public LEDStrip(int length, double brightness) {
        this(length, brightness, false);
    }

    public LEDStrip(int length, boolean isReversed) {
        this(length, 1.0, isReversed);
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

    public double getBrightness() {
        return brightness;
    }
    public void setBrightness(double brightness) {
        this.brightness = brightness;
    }

    public Color[] getLEDBuffer() {
        Color[] ledBuffer = new Color[length];
        if(isReversed) {
            for(int i = length - 1; i >= 0; i--) {
                ledBuffer[i] =
                    new Color(
                        colorBuffer[i].red * valueBuffer[i] * brightness,
                        colorBuffer[i].green * valueBuffer[i] * brightness,
                        colorBuffer[i].blue * valueBuffer[i] * brightness
                    );
            }
        }
        else {
            for(int i = 0; i < length; i++) {
                ledBuffer[i] =
                    new Color(
                        colorBuffer[i].red * valueBuffer[i] * brightness,
                        colorBuffer[i].green * valueBuffer[i] * brightness,
                        colorBuffer[i].blue * valueBuffer[i] * brightness
                    );
            } 
        }
        return ledBuffer;
    }

    public Color getLED(int index) {
        if(isReversed) {
            index = length - 1 - index;
        }
        return new Color(
            colorBuffer[index].red * valueBuffer[index] * brightness,
            colorBuffer[index].green * valueBuffer[index] * brightness,
            colorBuffer[index].blue * valueBuffer[index] * brightness
        );
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
        return pixelBuffer;    }

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
    }

    public void setColor(Color color, int index) {
        colorBuffer[index] = color;
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
    }

    public void setValue(double value) {
        for(int i = 0; i < length; i++) {
            valueBuffer[i] = value;
        }
    }

    public void setValue(double value, int index) {
        valueBuffer[index] = value;
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
    }
}
