package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.Subsystems.HorizontalExtension;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.Subsystems.VerticalLift;

@Autonomous
public class Auto_Test extends LinearOpMode {

    @Override
    public void runOpMode() {

        Pose2d beginPose = new Pose2d(0, 0, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        VerticalLift lift = new VerticalLift(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        HorizontalExtension intake = new HorizontalExtension(hardwareMap);


        TrajectoryActionBuilder path1 = drive.actionBuilder(beginPose)
                .strafeTo(new Vector2d(0, 25));

        Actions.runBlocking(intake.retract());
        telemetry.addData("frontLift pos", lift.frontLift.getCurrentPosition());
        telemetry.addData("backLift pos", lift.backLift.getCurrentPosition());

        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;


        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                path1.build(),
                                lift.raise(1000, 1)
                        ),
                        new ParallelAction(
                                lift.lower(0, .5),
                                claw.drop(lift.frontLift, 800)
                        )
                )
        );
    }
}
