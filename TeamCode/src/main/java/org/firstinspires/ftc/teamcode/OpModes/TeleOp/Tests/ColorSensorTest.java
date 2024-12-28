package org.firstinspires.ftc.teamcode.OpModes.TeleOp.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class ColorSensorTest extends LinearOpMode {
    private ColorRangeSensor colorRangeSensor;

    @Override
    public void runOpMode() throws InterruptedException {
        colorRangeSensor = hardwareMap.get(ColorRangeSensor.class, "color");

        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("distance", colorRangeSensor.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }
    }
}
