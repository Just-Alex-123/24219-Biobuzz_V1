package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class colorsensor extends OpMode {
    test_color testcolor = new test_color();
    test_color.DetectedColor detectedColor;

    @Override
    public void init() {
        testcolor.init(hardwareMap);
    }

    @Override
    public void loop() {
        detectedColor = testcolor.getDetectedColor(telemetry);
        telemetry.addData("Color Detected", detectedColor);
    }
}
