package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.Subsystems.HorizontalExtensionOLD;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.Subsystems.VerticalLift;

@Autonomous
public class SPECIMEN extends LinearOpMode {
    @Override
    public void runOpMode() {
        //instantiate subsystems
        Pose2d beginPose = new Pose2d(-15.5, 63, Math.toRadians(-90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        VerticalLift lift = new VerticalLift(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        HorizontalExtensionOLD intake = new HorizontalExtensionOLD(hardwareMap);

        //initialize trajectories
        TrajectoryActionBuilder path1 = drive.actionBuilder(beginPose)
                .strafeTo(new Vector2d(-7.7,33));

        TrajectoryActionBuilder park = path1.endTrajectory().fresh()
                .strafeTo(new Vector2d(-7.7,33))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-55, 60), Math.toRadians(90));

        TrajectoryActionBuilder path2 = path1.endTrajectory().fresh()

                //push 1
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-55, 58), Math.toRadians(-90)); //set
              /*  .setReversed(false)
                .splineToConstantHeading(new Vector2d(-30.5, 20), Math.toRadians(-90)) //turn
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(-48, 20), Math.toRadians(90)) //turn
                .lineToY(52.5, new TranslationalVelConstraint(15)) */

                //push 2
            /*    .setTangent(Math.toRadians(90))
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(-40, 20), Math.toRadians(-90)) //turn
                .setTangent(Math.toRadians(-90))
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(-58, 20), Math.toRadians(90)) //turn
                .lineToY(52.5); //push */

                //push 3
             /*   .setTangent(Math.toRadians(-90))
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(-52, 20), Math.toRadians(-90)) //turn
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(-62, 20), Math.toRadians(90)) //turn
                .lineToY(50); //push */


        //don't forget to run grab() action during init to maintain possession of specimen
        Actions.runBlocking(claw.grab());
        intake.servo.setPosition(.7);

        waitForStart();

        if (isStopRequested()) return;

        //Code
        Actions.runBlocking(
                new ParallelAction(
                        lift.move(),
                        new SequentialAction(
                                new ParallelAction(
                                        lift.SetSlidePos(670),
                                        path1.build()
                                ),
                                new SleepAction(.5),
                                lift.SetSlidePos(550),
                                new SleepAction(1),
                                claw.drop(),
                                new SleepAction(.5),
                                new ParallelAction(
                                        park.build(),
                                        new SequentialAction(
                                                new SleepAction(.2),
                                                claw.grab()
                                        ),
                                        new SequentialAction(
                                                new SleepAction(.5),
                                                lift.SetSlidePos(-20)
                                        )
                                )
                        )
                )
        );
    }
}

/*
.strafeTo(new Vector2d(-7.7,35))
        .setReversed(true)
                .splineToConstantHeading(new Vector2d(-37.5, 35.5), Math.toRadians(-90))
        .setReversed(false)
                .splineToConstantHeading(new Vector2d(-35.5, 0), Math.toRadians(-90))
        .setReversed(false)
                .splineToConstantHeading(new Vector2d(-45, 0), Math.toRadians(90))
        .lineToY(50)
                .setTangent(Math.toRadians(90))
        .setReversed(false)
                .splineToConstantHeading(new Vector2d(-45, 0), Math.toRadians(-90))
        .setTangent(Math.toRadians(-90))
        .setReversed(false)
                .splineToConstantHeading(new Vector2d(-56, 0), Math.toRadians(90))
        .lineToY(50)
                .setTangent(Math.toRadians(-90))
        .setReversed(false)
                .splineToConstantHeading(new Vector2d(-60, 0), Math.toRadians(90))
        .lineToY(55)
        */

/*.strafeTo(new Vector2d(-7.7,35))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-37.5, 35.5), Math.toRadians(-90))
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(-34, 0), Math.toRadians(-90))
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(-44, 0), Math.toRadians(90))
                .setTangent(Math.toRadians(90))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-45, 50), Math.toRadians(-90))
                .setTangent(Math.toRadians(-90))
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(-50, 0), Math.toRadians(90))
                .setTangent(Math.toRadians(90))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-55, 50), Math.toRadians(-90))
                .setTangent(Math.toRadians(-90))
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(-60, 0), Math.toRadians(90))
                .setTangent(Math.toRadians(90))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-61, 55), Math.toRadians(90))*/