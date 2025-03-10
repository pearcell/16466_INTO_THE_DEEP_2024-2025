package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.Subsystems.FourBarClaw;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.Subsystems.HorizontalExtension;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.Subsystems.HorizontalExtensionOLD;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.Subsystems.Swivel;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.Subsystems.VerticalLift;

@Autonomous
public class SAMPLE extends LinearOpMode {

    //swivel variable based on cv
    public double angleCV;

    @Override
    public void runOpMode() {
        //instantiate subsystems
        Pose2d beginPose = new Pose2d(38.35, 63.25, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        VerticalLift lift = new VerticalLift(hardwareMap);
        FourBarClaw claw = new FourBarClaw(hardwareMap);
        Swivel swivel = new Swivel(hardwareMap);
        HorizontalExtension intake = new HorizontalExtension(hardwareMap);


    //initialize trajectories
        //score preloaded sample
        TrajectoryActionBuilder scorePreloadA = drive.actionBuilder(beginPose)
                .strafeToLinearHeading(new Vector2d(50.2, 63.25), Math.toRadians(0));

        //sample 1 grab
        TrajectoryActionBuilder grab1A = scorePreloadA.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(55.4, 47), Math.toRadians(-112), new AngularVelConstraint(Math.toRadians(220)));

        //sample 1 score
        TrajectoryActionBuilder score1A = grab1A.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(58.9, 60.2), Math.toRadians(49.3));

        //sample 2 grab
        TrajectoryActionBuilder grab2A = score1A.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(55.4, 47), Math.toRadians(-85), new AngularVelConstraint(Math.toRadians(220)));

        //sample 2 score
        TrajectoryActionBuilder score2A = grab2A.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(58.9, 60.2), Math.toRadians(49.3));

        //sample 3 grab
        TrajectoryActionBuilder grab3A = score2A.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(55.4, 47), Math.toRadians(-60.0), new AngularVelConstraint(Math.toRadians(220)));

        //sample 3 score
        TrajectoryActionBuilder score3A = grab3A.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(58.9, 60.2), Math.toRadians(49.3));

        //park next to sub for level 1 ascent
        TrajectoryActionBuilder parkA = score3A.endTrajectory().fresh()
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(new Vector2d(38, 11), Math.toRadians(-90)), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(new Vector2d(23, 11), Math.toRadians(-180)), Math.toRadians(180));

    //prebuild
        Action scorePreload = scorePreloadA.build();

        Action grab1 = grab1A.build();

        Action score1 = score1A.build();

        Action grab2 = grab2A.build();

        Action score2 = score2A.build();

        Action grab3 = grab3A.build();

        Action score3 = score3A.build();

        Action park = parkA.build();

        //grab and retract
        Actions.runBlocking(claw.grab());
        intake.servo.setPosition(.73);

        telemetry.addData("frontLiftPos", lift.frontLift.getCurrentPosition());
        telemetry.addData("backLiftPos", lift.backLift.getCurrentPosition());
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        //Code
        Actions.runBlocking(
                new ParallelAction(
                        new SequentialAction(
                                //score preload
                                lift.SetSlidePos(1280),
                                new SleepAction(.4),
                                new ParallelAction(
                                        scorePreload,
                                        new SequentialAction(
                                                new SleepAction(.5),
                                                intake.extend(.65)
                                        ),
                                        new SequentialAction(
                                                new SleepAction(.7),
                                                claw.drop()
                                        )
                                ),
                                intake.retract(.73),

                                //grab 1
                                new ParallelAction(
                                        grab1,
                                        new SequentialAction(
                                                new SleepAction(.5),
                                                lift.SetSlidePos(70)
                                        ),
                                        new SequentialAction(
                                                new SleepAction(.6),
                                                intake.extend(.5),
                                                swivel.turn(.6)
                                        )
                                ),
                                new SleepAction(1),
                                lift.SetSlidePos(-60),
                                new SleepAction(.25),
                                claw.grab(),
                                new SleepAction(.25),
                                swivel.turn(.5),
                                intake.retract(.73),
                                lift.SetSlidePos(1280),
                                new SleepAction(.25),
                                //score 1
                                score1,
                                claw.drop(),
                                new SleepAction(.4),

                                //grab 2
                                new ParallelAction(
                                        grab2,
                                        new SequentialAction(
                                                new SleepAction(.5),
                                                lift.SetSlidePos(60)
                                        ),
                                        new SequentialAction(
                                                new SleepAction(.6),
                                                intake.extend(.52),
                                                swivel.turn(.45)
                                        )
                                ),
                                new SleepAction(1),
                                lift.SetSlidePos(-60),
                                new SleepAction(.25),
                                claw.grab(),
                                new SleepAction(.25),
                                swivel.turn(.5),
                                intake.retract(.73),
                                lift.SetSlidePos(1280),
                                new SleepAction(.4),
                                //score 2
                                score2,
                                claw.drop(),
                                new SleepAction(.25),

                                //grab 3
                                new ParallelAction(
                                        grab3,
                                        new SequentialAction(
                                                new SleepAction(.5),
                                                lift.SetSlidePos(60)
                                        ),
                                        new SequentialAction(
                                                new SleepAction(.6),
                                                intake.extend(.45),
                                                swivel.turn(.4)
                                        )
                                ),
                                new SleepAction(1),
                                lift.SetSlidePos(-60),
                                new SleepAction(.25),
                                claw.grab(),
                                new SleepAction(.25),
                                swivel.turn(.5),
                                intake.retract(.73),
                                lift.SetSlidePos(1280),
                                new SleepAction(.4),
                                //score 3
                                score3,
                                claw.drop(),
                                new SleepAction(.25),

                                //park
                                new ParallelAction(
                                        park,
                                        new SequentialAction(
                                                new SleepAction(.5),
                                                lift.SetSlidePos(370)
                                        )
                                ),
                                intake.extend(.6)


                        ),
                        lift.move()
                )
        );
    }
}
