package org.firstinspires.ftc.teamcode.OpModes.TeleOp.Tests;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class armTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo servoArm = hardwareMap.servo.get("servoArm");



        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();
        waitForStart();
        double rest = 0.5;
        double out = 0.2;
        double armclickcount = 0;
        if (isStopRequested()) return;
        while (opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);
            if (currentGamepad1.x && !previousGamepad1.x) {
                if (armclickcount % 2 == 1) {
                    servoArm.setPosition(out);
                    armclickcount = armclickcount + 1;

                } else if (armclickcount % 2 == 0) {
                    servoArm.setPosition(rest);
                    armclickcount = armclickcount + 1;
                }
                    telemetry.addData("Arm Position", servoArm.getPosition());
                     telemetry.addData("Arm Click Count", armclickcount);
                    telemetry.update();

            }
        }
    }
}
