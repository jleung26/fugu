package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystem.ExtendingOuttake;
import org.firstinspires.ftc.teamcode.Subsystem.HorizontalSlides;
import org.firstinspires.ftc.teamcode.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.Subsystem.VerticalSlides;
import org.firstinspires.ftc.teamcode.Util.RobotHardware;

import java.util.ArrayList;
import java.util.List;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;


@Autonomous(name = "Submersible Intake Test", group = "A")
public class SubIntakeTest extends OpMode {
    // declaring subsystems
    RobotHardware robotHardware = new RobotHardware();
    VerticalSlides verticalSlides = new VerticalSlides();
    HorizontalSlides horizontalSlides = new HorizontalSlides();
    ExtendingOuttake outtake = new ExtendingOuttake();
    Intake intake = new Intake();
    Follower follower;
    Timer pathTimer, actionTimer, opmodeTimer;

    // booleans
    // team color
    // spec/sample
    boolean redAlliance = true;
    Intake.IntakeChamberState COLOR_TO_REJECT;

    // Action stuff
    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();

    // bulk cache reading
    private List<LynxModule> allHubs;

    // adds ability to graph telemetry values
    private MultipleTelemetry dashboardTelemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());

    // loop time tracking
    private ElapsedTime elapsedtime;

    private int pathState;

    /* Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     * Lets assume the Robot is facing the human player and we want to score in the bucket */

    // TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO
    // no visualizer access rn, so need to determine poses
    // also modify extendPartial fraction, also find 42" extension limit
    // use 42" limit in visualizer, might have to input 84" bot lol

    /** Start Pose of our robot */
    private final Pose startPose = new Pose(62, 98, Math.toRadians(270));
    // X: bot against wall
    // Y: along closest edge of tile to bucket, just before covering the foam connecting teeth
    // back of bot towards bucket

    /** Bucket Scoring Pose */
    private final Pose scorePose = new Pose(15, 129, Math.toRadians(315)); // TODO: tuned, but can make more optimal

    /** First Sample from the Spike Mark */
    private final Pose pickup1Pose = new Pose(16, 128, Math.toRadians(0)); // TODO: tuned, but can make more optimal
    // TODO: (e.g. with less movement from score, and turning instead) if need to save some time
    // old, very consistent pose, switch back if unable to tune new pos: 15, 128, Math.toRadians(0)

    /** Second Sample from the Spike Mark */
    private final Pose pickup2Pose = new Pose(16, 133, Math.toRadians(0)); // tuned

    /** Third Sample from the Spike Mark */
    private final Pose pickup3Pose = new Pose(25, 120, Math.toRadians(55)); // tuned

    /** Park Pose for our robot, after we do all of the scoring. */
    private final Pose parkPose = new Pose(62, 98, Math.toRadians(270)); // tuned

    private final Pose scoreControlPose = new Pose(68, 110, Math.toRadians(999)/* heading unused*/); // done

    private final Pose sub2Pose = new Pose(startPose.getX()+5, 98, Math.toRadians(270));

    private final Pose sub3Pose = new Pose(sub2Pose.getX()+5, 98, Math.toRadians(270));

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private PathChain scorePreload, grabPickup1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3, scoreFrom1, sub2, scoreFrom2, sub3, scoreFrom3, repeatSub;


    public void buildPaths() {

        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(scorePose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();
        /* Here is an example for Constant Interpolation
        scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. Straight line. */
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup1Pose)))
                .setConstantHeadingInterpolation(pickup1Pose.getHeading())
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup1Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                // TODO: maybe constant heading here?
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup2Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup2Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup3Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup3Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our park path. We are using a BezierCurve with 3 points, which is a curved line that is curved based off of the control point */
        scoreFrom1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(startPose), /* Control Point */ new Point(scoreControlPose), new Point(scorePose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        sub2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(sub2Pose)))
                .setConstantHeadingInterpolation(sub2Pose.getHeading())
                .build();

        scoreFrom2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(sub2Pose), /* Control Point */ new Point(scoreControlPose), new Point(scorePose)))
                .setLinearHeadingInterpolation(sub2Pose.getHeading(), scorePose.getHeading())
                .build();

        sub3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sub2Pose), new Point(sub3Pose)))
                .setConstantHeadingInterpolation(sub3Pose.getHeading())
                .build();

        scoreFrom3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(sub3Pose), /* Control Point */ new Point(scoreControlPose), new Point(scorePose)))
                .setLinearHeadingInterpolation(sub3Pose.getHeading(), scorePose.getHeading())
                .build();

        repeatSub = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sub3Pose), new Point(startPose)))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                subIntakeAction();
                if (intake.chamberState != COLOR_TO_REJECT && intake.chamberState != Intake.IntakeChamberState.EMPTY) {
                    retractIntakeAction();
                    setPathState(13);
                }
                else if (pathTimer.getElapsedTimeSeconds() > 2) {
                    retractIntakeAction();
                    setPathState(2);
                }
                break;
            case 2:
                follower.followPath(sub2, true);
                setPathState(3);
            case 3:
                if (!follower.isBusy()) {
                    subIntakeAction();
                    if (intake.chamberState != COLOR_TO_REJECT && intake.chamberState != Intake.IntakeChamberState.EMPTY) {
                        retractIntakeAction();
                        setPathState(14);
                    }
                    else if (pathTimer.getElapsedTimeSeconds() > 2) {
                        retractIntakeAction();
                        setPathState(4);
                    }
                }
                break;
            case 4:
                follower.followPath(sub3, true);
                setPathState(5);
            case 5:
                if (!follower.isBusy()) {
                    subIntakeAction();
                    if (intake.chamberState != COLOR_TO_REJECT && intake.chamberState != Intake.IntakeChamberState.EMPTY) {
                        retractIntakeAction();
                        setPathState(15);
                    }
                    else if (pathTimer.getElapsedTimeSeconds() > 2) {
                        setPathState(-1);
                    }
                }
                break;
            case 13:
                follower.followPath(scoreFrom1, true);
                setPathState(16);
                break;
            case 14:
                follower.followPath(scoreFrom2, true);
                setPathState(16);
                break;
            case 15:
                follower.followPath(scoreFrom3, true);
                setPathState(16);
                break;
            case 16:
                if (!follower.isBusy()) {
                    setPathState(-1);
                }
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        // init subsystems
        elapsedtime = new ElapsedTime();
        robotHardware.initialize(this);
        verticalSlides.initialize(this, robotHardware, true);
        horizontalSlides.initialize(this, robotHardware, true);
        outtake.initialize(this, robotHardware);
        intake.initialize(this, robotHardware);

        // bulk cache reading
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) { hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL); }

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        buildPaths();

        // ready to go
        outtake.closeClawLoose();
        outtake.toStow();
        intake.flipUp();
    }

    @Override
    public void init_loop() {
        // very important to select for when eventually >4 sample
        if (gamepad1.b || gamepad2.b) {
            redAlliance = true;
        } else if (gamepad1.x || gamepad2.x) {
            redAlliance = false;
        }

        COLOR_TO_REJECT = (redAlliance ? Intake.IntakeChamberState.BLUE : Intake.IntakeChamberState.RED);
        telemetry.addData("red alliance? ", redAlliance);
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void loop() {
        // action and bulk caching loops
        // clearing bulk cache
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        // for RR Action execution
        TelemetryPacket packet = new TelemetryPacket();
        List<Action> newActions = new ArrayList<>();
        for (Action action : runningActions) {
            if (action.run(packet)) { // actually running actions
                newActions.add(action); // if failed (run() returns true), try again next loop
            }
        }
        runningActions = newActions;

        // loops
        verticalSlides.operate();
        horizontalSlides.operateAuto();
        intake.operateColorChecking();

        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub
        telemetry.addLine("\n States");
        telemetry.addData("path state: ", pathState);
        telemetry.addData("isBusy? ", follower.isBusy());
        telemetry.addData("pinpoint cooked? ", follower.isLocalizationNAN());
        telemetry.addData("robot stuck? ", follower.isRobotStuck());
        telemetry.addData("path timer seconds", pathTimer.getElapsedTimeSeconds());

        telemetry.addLine("\n Pose");
        telemetry.addData("x: ", follower.getPose().getX());
        telemetry.addData("y: ", follower.getPose().getY());
        telemetry.addData("heading: ", follower.getPose().getHeading());

        telemetry.addData("\n Loop Times: ", elapsedtime.milliseconds());
        elapsedtime.reset();
    }

    // Abstracted Action Methods
    public void allToScoreAction() {
        // prep to score
        runningActions.add(new SequentialAction(
                new InstantAction(() -> verticalSlides.raiseToHighBucket()),
                new SleepAction(0.3),
                new InstantAction(() -> outtake.toVert()),
                new InstantAction(() -> outtake.armExtend.extendToScoreBucket()),
                new SleepAction(0.5), // TODO: tune this based on extension speed 0.3+0.5, probably only change 0.5
                new InstantAction(() -> outtake.toScoreBucket())
        ));
    }

    public void transferAndPrepToScoreAction() {
        // full transfer and prep to score sequence
        runningActions.add(new SequentialAction(
                new InstantAction(() -> intake.setIntake(0.4)), // push sample all the way in, kinda jank, maybe not necessary
                new InstantAction(() -> outtake.toTransfer()),
                new SleepAction(0.2), // TODO: play around with timings
                new InstantAction(() -> outtake.closeClawTight()),
                new SleepAction(0.3),
                new InstantAction(() -> outtake.toStow()),
                new SleepAction(0.1),
                new InstantAction(() -> intake.setIntake(0)),
                new InstantAction(() -> verticalSlides.raiseToHighBucket()),
                new SleepAction(0.3),
                new InstantAction(() -> outtake.toVert())
        ));
    }

    public void depositSampleAction() {
        // deposit in bucket, then retract all
        runningActions.add(new SequentialAction(
                new InstantAction(() -> outtake.toScoreBucket()),
                new SleepAction(0.3), // TODO: tune this based on extension speed 0.3+0.5, probably only to lower 0.5
                new InstantAction(() -> outtake.openClaw()),
                new SleepAction(0.2),
                new ParallelAction(
                        new InstantAction(() -> outtake.toStow()),
                        new InstantAction(() -> verticalSlides.retract())
                )
        ));
    }

    public void subIntakeAction() {
        runningActions.add(new SequentialAction(
                new InstantAction(() -> horizontalSlides.extendPartial()),
                new InstantAction(() -> intake.dropDown()),
                new InstantAction(() -> intake.intake()),
                new InstantAction(() -> horizontalSlides.extend())
        ));
    }

    public void retractIntakeAction() {
        runningActions.add(new SequentialAction(
                new InstantAction(() -> horizontalSlides.retract()),
                new InstantAction(() -> intake.flipUp()),
                new InstantAction(() -> intake.idle())
        ));
    }

}
