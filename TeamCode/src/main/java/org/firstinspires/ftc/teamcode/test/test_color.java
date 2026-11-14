package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class test_color {
    private RevColorSensorV3 cs;

    public enum DetectedColor {
        red,
        blue,
        yellow,
        unknown
    }
    public void init(HardwareMap hwmap){
        cs = hwmap.get(RevColorSensorV3.class,"cs");
        cs.setGain(15);
    }
    public DetectedColor getDetectedColor(Telemetry telemetry) {
        NormalizedRGBA colors = cs.getNormalizedColors();

        float normRed, normGreen, normBlue;
        normRed = colors.red / colors.alpha;
        normGreen = colors.green / colors.alpha;
        normBlue = colors.blue / colors.alpha;
        telemetry.addData("red", normRed);
        telemetry.addData("green", normGreen);
        telemetry.addData("blue", normBlue);


//        blue > 0.2 red < 0.12

        //red ~~ red > 0.21 blue < 0.15

        if (normRed > 0.19 && normBlue < 0.17) {
            return DetectedColor.red;

        } else if (normBlue > 0.2 && normRed < 0.12){
            return DetectedColor.blue;
        } else {
            return DetectedColor.unknown;
        }
    }


}

