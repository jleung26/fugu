//package org.firstinspires.ftc.teamcode.Util;
//import com.qualcomm.hardware.broadcom.BroadcomColorSensor;
//import com.qualcomm.hardware.rev.RevColorSensorV3;
//import com.qualcomm.robotcore.hardware.ColorRangeSensor;
//import com.qualcomm.robotcore.hardware.DistanceSensor;
//import com.qualcomm.robotcore.hardware.HardwareDevice;
//import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
//import com.qualcomm.robotcore.hardware.NormalizedRGBA;
//import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
//import com.qualcomm.robotcore.util.Range;
//import com.qualcomm.robotcore.util.RobotLog;
//import com.qualcomm.robotcore.util.TypeConversion;
//
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//
//import com.qualcomm.hardware.rev.RevColorSensorV3;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//public class CustomRevColorSensorV3 extends RevColorSensorV3 {
//    long lastRead = 0, measurementDelay = 7;
//    int red = 0, green = 0, blue = 0, alpha = 0;
//
//    public CustomRevColorSensorV3(HardwareMap hardwareMap, String deviceName) {
//        super(hardwareMap.get(I2cDeviceSynchSimple.class, deviceName));
//    }
//    public RevColorSensorV3Ex(I2cDeviceSynchSimple deviceClient)
//    {
//        super(deviceClient);
//    }
//
//    // Method to set PSRate to 400kHz using the built-in function
//    public void configurePSRate() {
//        setPSRateAndRes(PSResolution.RES_11BIT, PSMeasurementRate.RATE_400KHZ);
//    }
//
//    // Method to read 12 bytes from PS_DATA in one I2C transaction
//    public byte[] readColorAndDistance() {
//        return this.
//    }
//
//    public void update() {
//        long now = System.currentTimeMillis();
//        if(now - lastRead > measurementDelay) {
//            byte[] data;
//
//            // Read red, green and blue values
//            final int cbRead = 12;
//            data = read(Register.PS_DATA, cbRead);
//
//            final int dib = 0;
//            this.green = TypeConversion.unsignedShortToInt(TypeConversion.byteArrayToShort(data, dib, ByteOrder.LITTLE_ENDIAN));
//            this.blue = Range.clip((int) (1.55 * TypeConversion.unsignedShortToInt(TypeConversion.byteArrayToShort(
//                    data, dib + 3, ByteOrder.LITTLE_ENDIAN))), 0, 65535);
//            this.red = Range.clip((int) (1.07 * TypeConversion.unsignedShortToInt(TypeConversion.byteArrayToShort(
//                    data, dib + 6, ByteOrder.LITTLE_ENDIAN))), 0, 65535);
//
//            this.alpha = (this.red + this.green + this.blue) / 3;
//
//            // normalize to [0, 1]
//            this.colors.red = Range.clip(((float) this.red * getGain()) / parameters.colorSaturation, 0f, 1f);
//            this.colors.green = Range.clip(((float) this.green * getGain()) / parameters.colorSaturation, 0f, 1f);
//            this.colors.blue = Range.clip(((float) this.blue * getGain()) / parameters.colorSaturation, 0f, 1f);
//
//            // apply inverse squared law of light to get readable brightness value, stored in alpha channel
//            // scale to 65535
//            float avg = (float) (this.red + this.green + this.blue) / 3;
//            this.colors.alpha = (float) (-(65535f / (Math.pow(avg, 2) + 65535)) + 1);
//
//            lastRead = System.currentTimeMillis();
//        }
//    }
//}
