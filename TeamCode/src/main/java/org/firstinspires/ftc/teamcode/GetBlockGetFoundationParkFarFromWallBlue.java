package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

@Autonomous(name="GetBlockGetFoundationParkFarFromWallBlue", group="Autonomous")

public class GetBlockGetFoundationParkFarFromWallBlue extends LinearOpMode{
    DriveTrain robot = new DriveTrain();
    Intake intake = new Intake();
    FoundationMechanism foundation = new FoundationMechanism();
    BlockArm blockArm = new BlockArm();
    ElapsedTime time = new ElapsedTime();
    boolean found = false;
    private String color = "";

    private final double CLEAR_WALL_DISTANCE = 10; //was 15. still need to tune
    private final double INITIAL_STRAFE_DISTANCE = 28.5; //TODO Tune
    private final double STRAFE_TO_BLOCK = 3.5; //TODO Tune
    private final double DIST_TO_MOVE_UP = 60; //TODO Tune

    private final double DRIVE_SPEED = 0.8;
    private final double TURN_SPEED = 0.5;
    private final double STRAFE_SPEED = 0.5;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap); //x: 48 30 y: 54 54. start facing backwards
        intake.init(hardwareMap);
        foundation.init(hardwareMap);
        blockArm.init(hardwareMap);
        blockArm.deactivateClamp();

        waitForStart();

        robot.mecanumStrafe(STRAFE_SPEED, INITIAL_STRAFE_DISTANCE, MovementDirection.RIGHT, 0);
        stateZero();

        activeScanning();
        color = blockArm.getColor();
        telemetry.addData("color", color);
        telemetry.update();
        // sleep(100);
        if (color.equals("black")) {
            stateA();
        }

        stateZero();
        robot.mecanumDriveStraightAlongZero(DRIVE_SPEED,8,0); //0.6
        if (!found) {
            activeScanning();
            color = blockArm.getColor();
            telemetry.addData("color", color);
            telemetry.update();
            // sleep(100);
            if (color.equals("black")) {
                stateA();
            }
        }

        stateZero();
        robot.mecanumDriveStraightAlongZero(DRIVE_SPEED,8,0); //0.6
        if (!found) {
            activeScanning();
            color = blockArm.getColor();
            telemetry.addData("color", color);
            // sleep(100);
            if (color.equals("black")) {
                stateA();
            }
        }

        stateB();
        robot.mecanumDriveStraightAlongZero(DRIVE_SPEED,DIST_TO_MOVE_UP,0); //0.8
        stateC();

        telemetry.update();
        robot.turnRobot(TURN_SPEED, 90); //turn 90 counter-clockwise
        robot.mecanumDriveStraightAlongZero(-DRIVE_SPEED,-10,90); //-0.5
        foundation.clampFoundation();
        sleep(300);
        robot.mecanumDriveStraightAlongZero(DRIVE_SPEED,33,90); //0.8
        robot.turnRobot(TURN_SPEED, 180); //turn 90 counter-clockwise
        robot.mecanumDriveStraightAlongZero(-DRIVE_SPEED,-10,180); //-0.8
        foundation.releaseFoundation();
        robot.mecanumStrafe(STRAFE_SPEED, 10, MovementDirection.LEFT, 0);
        robot.mecanumDriveStraightAlongZero(DRIVE_SPEED,40,180); //0.5

    }

    public void stateZero() {
        if (!found) {
            blockArm.scanPosition();
            sleep(700);
        }
    }

    public void activeScanning() {
        blockArm.activateArm();
        blockArm.deactivateClamp();
        sleep(700);
    }

    public void stateA() {
        blockArm.scanPosition();
        sleep(700);
        robot.mecanumStrafe(STRAFE_SPEED, STRAFE_TO_BLOCK, MovementDirection.RIGHT, 0);
        blockArm.activateArm();
        sleep(1000);
        blockArm.activateClamp();
        sleep(1000);
        found = true;
        blockArm.armUp();
        robot.mecanumStrafe(STRAFE_SPEED, CLEAR_WALL_DISTANCE, MovementDirection.LEFT, 0);
    }

    public void stateB() {

    }

    public void stateC() {
        robot.mecanumStrafe(STRAFE_SPEED, CLEAR_WALL_DISTANCE, MovementDirection.RIGHT, 0);
        blockArm.activateArm();
        sleep(700);
        blockArm.deactivateClamp();
        sleep(700);
        blockArm.armUp();
        sleep(700);
    }
}