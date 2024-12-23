package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.OpModes.Autonomous.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.Subsystems.VerticalLift;

@Autonomous
public class BLUE_LEFT extends LinearOpMode {
    @Override
    public void runOpMode() {
        VerticalLift verticalLift = new VerticalLift(hardwareMap);
        Claw claw = new Claw(hardwareMap);

    }
}
