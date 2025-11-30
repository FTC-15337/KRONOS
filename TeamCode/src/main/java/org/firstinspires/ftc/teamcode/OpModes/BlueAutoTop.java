package org.firstinspires.ftc.teamcode.OpModes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.PedroPathing.Constants;

@Autonomous
public class BlueAutoTop extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer;

    public enum PathState {
        // START POSITION_END POSITION
        // DRIVE > MOVEMENT STATE
        // SHOOT > ATTEMPT TO SCORE THE ARTIFACT
        DRIVE_STARTPOS_SHOOT_POS,
        SHOOT_PRELOAD,

        DRIVE_SHOOTPOS_BPICKUP // BPICKUP is begin pickup and EPICKUP is end pickup
    }

    PathState pathState;

    private final Pose startPose = new Pose(21.843640606767796, 122.15635939323221, Math.toRadians(130));
    private final Pose shootPose = new Pose(47.55192532088682, 95.77596266044341, Math.toRadians(130));

    private final Pose beginPickup1 = new Pose(47.71995332555426, 84.18203033838974, Math.toRadians(180));

    private PathChain driveStartPosShootPos, driveShootPosBPickupPos;

    public void buildPaths() {
        // put in coordinates for starting pose > ending pose
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setConstantHeadingInterpolation(Math.toRadians(130))
                .build();

        driveShootPosBPickupPos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, beginPickup1))
                .setLinearHeadingInterpolation(shootPose.getHeading(), beginPickup1.getHeading())
                .build();
    }

    public void statePathUpdate() {
        switch(pathState){
            case DRIVE_STARTPOS_SHOOT_POS:
                follower.followPath(driveStartPosShootPos, true);
                telemetry.addLine("Going to shoot");
                setPathState(PathState.SHOOT_PRELOAD); // reset timer and make a new state
                break;
            case SHOOT_PRELOAD:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 5){ // not using sleep as it stops the whole thread so using timer instead
                    // TODO add shooter to shoot
                    telemetry.addLine("Shooting");
                    // transition to next state
                    follower.followPath(driveShootPosBPickupPos, true);
                    setPathState(PathState.DRIVE_SHOOTPOS_BPICKUP);
                }
                break;
            case DRIVE_SHOOTPOS_BPICKUP:
                if(!follower.isBusy()) {
                    telemetry.addLine("About to start pickup 1");
                }
            default:
                telemetry.addLine("Not in any state");
                break;
        }
    }

    public void setPathState(PathState newState){
        pathState = newState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathState = PathState.DRIVE_STARTPOS_SHOOT_POS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        // TODO add any other subsystems

        buildPaths();
        follower.setPose(startPose);
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();
    }
}