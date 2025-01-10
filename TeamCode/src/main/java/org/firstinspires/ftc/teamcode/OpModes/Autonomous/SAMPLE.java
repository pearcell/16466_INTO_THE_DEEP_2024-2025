package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.Subsystems.HorizontalExtension;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.Subsystems.VerticalLift;

@Autonomous
public class SAMPLE extends LinearOpMode {
    @Override
    public void runOpMode() {
        //instantiate subsystems
        Pose2d beginPose = new Pose2d(15.5, 63, Math.toRadians(-90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        VerticalLift lift = new VerticalLift(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        HorizontalExtension intake = new HorizontalExtension(hardwareMap);

    //initialize trajectories
        //score preloaded sample
        TrajectoryActionBuilder scorePreload = drive.actionBuilder(beginPose)
                .splineToLinearHeading(new Pose2d(new Vector2d(57.9, 59.2), Math.toRadians(49.3)), Math.toRadians(45), new TranslationalVelConstraint(20));

        //sample 1 grab
        TrajectoryActionBuilder grab1 = scorePreload.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(49.8, 51.6), Math.toRadians(-90), new AngularVelConstraint(Math.toRadians(90)));

        //sample 1 score
        TrajectoryActionBuilder score1 = grab1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(57.9, 59.2), Math.toRadians(49.3));


        //sample 2 grab
        TrajectoryActionBuilder grab2 = score1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(49.2, 50.3), Math.toRadians(-66.6));

        //sample 2 score
        TrajectoryActionBuilder score2 = grab2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(57.9, 59.2), Math.toRadians(49.3));

        //sample 3 grab
        TrajectoryActionBuilder grab3 = score2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(53.9, 47.7), Math.toRadians(-56.4));

        //sample 3 score
        TrajectoryActionBuilder score3 = grab3.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(57.9, 59.2), Math.toRadians(49.3));

        //park next to sub for level 1 ascent
        TrajectoryActionBuilder park = score3.endTrajectory().fresh()
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(new Vector2d(38, 11), Math.toRadians(-90)), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(new Vector2d(25, 11), Math.toRadians(180)), Math.toRadians(180));


        //don't forget to run grab() action during init to maintain possession of sample
        Actions.runBlocking(claw.grab());
        intake.servo.setPosition(.48);

        waitForStart();

        if (isStopRequested()) return;

        //Code

        Actions.runBlocking(
                new ParallelAction(
                        new SequentialAction(
                                //score preload
                                lift.SetSlidePos(3200),
                                scorePreload.build(),
                                claw.drop(),
                                new SleepAction(.5),
                                new ParallelAction(
                                        claw.grab(),
                                        grab1.build(),
                                        new SequentialAction(
                                                new SleepAction(.5),
                                                lift.SetSlidePos(0)
                                        )
                                ),
                                //grab 1
                                claw.drop(),
                                intake.extend(),
                                new SleepAction(1),
                                claw.grab(),
                                new SleepAction(1),
                                intake.retract(),
                                new SleepAction(1),
                                //score 1
                                lift.SetSlidePos(3200),
                                new SleepAction(1),
                                score1.build(),
                                claw.drop(),
                                new SleepAction(.5),
                                new ParallelAction(
                                        claw.grab(),
                                        grab2.build(),
                                        new SequentialAction(
                                                new SleepAction(.5),
                                                lift.SetSlidePos(0)
                                        )
                                ),

                                //grab 2
                                claw.drop(),
                                intake.extend(),
                                new SleepAction(1),
                                claw.grab(),
                                new SleepAction(1),
                                intake.retract(),
                                new SleepAction(1),
                                //score 2
                                lift.SetSlidePos(3200),
                                new SleepAction(1),
                                score2.build(),
                                claw.drop(),
                                new SleepAction(.5),
                                new ParallelAction(
                                        claw.grab(),
                                        grab3.build(),
                                        new SequentialAction(
                                                new SleepAction(.5),
                                                lift.SetSlidePos(0)
                                        )
                                ),

                                //grab 3
                                claw.drop(),
                                intake.extend(),
                                new SleepAction(1),
                                claw.grab(),
                                new SleepAction(1),
                                intake.retract(),
                                new SleepAction(1),
                                //score 3
                                lift.SetSlidePos(3200),
                                new SleepAction(1),
                                score3.build(),
                                claw.drop(),
                                new SleepAction(.5),
                                new ParallelAction(
                                        claw.grab(),
                                        park.build(),
                                        new SequentialAction(
                                                new SleepAction(.5),
                                                lift.SetSlidePos(0)
                                        )
                                )
                        ),


                        lift.move()
                )
        );

        /*new SequentialAction(
                scorePreload.build(),
                new SleepAction(1),
                grab1.build(),
                new SleepAction(1),
                score1.build(),
                new SleepAction(1),
                grab2.build(),
                new SleepAction(1),
                score2.build(),
                new SleepAction(1),
                grab3.build(),
                new SleepAction(1),
                score3.build(),
                new SleepAction(1),
                park.build()
        ) */
        /*Actions.runBlocking(
                new SequentialAction(
                        //score block 1
                        new ParallelAction(
                                score.build(),
                                lift.raise(1000, .5),
                                claw.drop(lift.frontLift, 1500)
                        ),
                        //score block 2
                        new ParallelAction(
                                grab1.build(),
                                lift.lower(0, .5),
                                intake.extend(lift.frontLift, 50)
                        ),
                        new ParallelAction(
                                claw.grab(),
                                intake.retract()
                        ),
                        new ParallelAction(
                                score.build(),
                                lift.raise(1000, 1),
                                claw.drop(lift.frontLift, 1500)
                        ),
                        //score block 3
                        new ParallelAction(
                                grab2.build(),
                                lift.lower(0, .5),
                                intake.extend(lift.frontLift, 50)
                        ),
                        new ParallelAction(
                                claw.grab(),
                                intake.retract()
                        ),
                        new ParallelAction(
                                score.build(),
                                lift.raise(1000, 1),
                                claw.drop(lift.frontLift, 1500)
                        ),
                        //score block 4
                        new ParallelAction(
                                grab3.build(),
                                lift.lower(0, .5),
                                intake.extend(lift.frontLift, 50)
                        ),
                        new ParallelAction(
                                claw.grab(),
                                intake.retract()
                        ),
                        new ParallelAction(
                                score.build(),
                                lift.raise(1000, 1),
                                claw.drop(lift.frontLift, 1500)
                        ),
                        //score park
                        new ParallelAction(
                                park.build(),
                                lift.lower(400, .5)
                        )
                )
        ); */
    }
}
