package org.firstinspires.ftc.teamcode.OpModes.TeleOp.Tests;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp

public class clawTest extends LinearOpMode {
    @Override

    public void runOpMode() throws InterruptedException {
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();
        Servo servoClaw = hardwareMap.servo.get("servoClaw");

        waitForStart();
       double closed = 0.15;
       double open = 0;
       double clickcount = 0;

        if (isStopRequested()) return;
        while (opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);
            if (currentGamepad1.b && !previousGamepad1.b) {
                if ( clickcount % 2 == 1) {
                    servoClaw.setPosition(open);
                    clickcount = clickcount + 1;

                } else if( clickcount % 2 == 0) {
                    servoClaw.setPosition(closed);
                    clickcount = clickcount + 1;
                }



            }
            telemetry.addData("Claw Position", servoClaw.getPosition());
            telemetry.addData("Click Count", clickcount);
            telemetry.update();
        }
    }
}