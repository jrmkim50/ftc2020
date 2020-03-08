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

@Autonomous(name="TwoBlockAutonomousRed", group="Autonomous")

public class TwoBlockAutonomousRed extends LinearOpMode{
    DriveTrain robot = new DriveTrain();
    Intake intake = new Intake();
    FoundationMechanism foundation = new FoundationMechanism();
    BlockArm blockArm = new BlockArm();
    ElapsedTime time = new ElapsedTime();
    boolean found = false;
    private String color = "";
    private String position = "zero";

    private final double CLEAR_WALL_DISTANCE = 10; //Was 15. still tuning
    private final double INITIAL_STRAFE_DISTANCE = 28.5; //TODO Tune
    private final double STRAFE_TO_BLOCK = 3.5; //TODO Tune
    private final double DIST_TO_MOVE_UP = 80; //TODO Tune

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

        robot.mecanumDriveStraightAlongZero(DRIVE_SPEED,13,0); //0.6

        robot.mecanumStrafe(STRAFE_SPEED, INITIAL_STRAFE_DISTANCE, MovementDirection.RIGHT, 0);
        stateZero();

        activeScanning();
        color = blockArm.getColor();
        telemetry.addData("color", color);
        telemetry.update();
        if (color.equals("black")) {
            stateA();
            position = "two";
        }

        stateZero();
        robot.mecanumDriveStraightAlongZero(-DRIVE_SPEED,-10,0); //-0.6
        if (!found) {
            activeScanning();
            color = blockArm.getColor();
            telemetry.addData("color", color);
            telemetry.update();
            if (color.equals("black")) {
                stateA();
                position = "one";
            }
        }

        stateZero();
        robot.mecanumDriveStraightAlongZero(-DRIVE_SPEED,-10,0); //-0.6
        if (!found) {
            activeScanning();
            color = blockArm.getColor();
            telemetry.addData("color", color);
            if (color.equals("black")) {
                stateA();
                position = "zero";
            }
        }

        stateB();
        robot.mecanumDriveStraightAlongZero(-DRIVE_SPEED,-1*DIST_TO_MOVE_UP,3); //-0.8
        stateC();

        stateD();
        stateZero();
        if (position.equals("zero")) {
            activeScanning();
            color = blockArm.getColor();
            if (color.equals("black")) {
                stateA();
            }
        }

        stateZero();
        robot.mecanumDriveStraightAlongZero(DRIVE_SPEED,10,0); //0.6
        if (position.equals("one")) {
            activeScanning();
            color = blockArm.getColor();
            if (color.equals("black")) {
                stateA();
            }
        }


        robot.mecanumDriveStraightAlongZero(-DRIVE_SPEED,-20,0); //0.6
        robot.mecanumDriveStraightAlongZero(-DRIVE_SPEED,-20,0); //0.6
        stateB();
        robot.mecanumDriveStraightAlongZero(-DRIVE_SPEED,-DIST_TO_MOVE_UP,0); //0.8
        stateC();

        retrieveFoundation();
    }

    public void retrieveFoundation() {
        robot.turnRobot(TURN_SPEED, 90); //turn 90 counter-clockwise
        robot.mecanumDriveStraightAlongZero(-DRIVE_SPEED,-10,90); //-0.5
        foundation.clampFoundation();
        sleep(100);
        robot.mecanumDriveStraightAlongZero(DRIVE_SPEED,33,90); //0.8
        robot.turnRobot(TURN_SPEED, 0); //turn 90 counter-clockwise
        robot.mecanumDriveStraightAlongZero(-DRIVE_SPEED,-10,0); //-0.8
        foundation.releaseFoundation();
        robot.mecanumDriveStraightAlongZero(DRIVE_SPEED,40,0); //0.5
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

    public void stateD() {
        robot.mecanumStrafe(STRAFE_SPEED, CLEAR_WALL_DISTANCE, MovementDirection.LEFT, 0);
        robot.mecanumDriveStraightAlongZero(DRIVE_SPEED,DIST_TO_MOVE_UP,0); //0.8
        robot.mecanumDriveStraightAlongZero(DRIVE_SPEED,30,0); //0.6
    }
}