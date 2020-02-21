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
    ElapsedTime time = new ElapsedTime();
    String color;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap); //x: -18 -36 y: -54 -54. start with front of robot facing bridge
        intake.init(hardwareMap);
        foundation.init(hardwareMap);

        waitForStart();

        robot.mecanumStrafe(0.5, 20,MovementDirection.RIGHT, 0); //x: -18 -36 y: -34 -34
        //colorSensor.getColor()
        if (color.equals("black")) {
            //robotArm.bringUp()
            //robotArm.bringDown()
            //robotArm.bringUp()
            robot.mecanumDriveStraightAlongZero(0.5, 66, 0); //x: 48 30 y: -34 -34
            //robotArm.bringDown()
            //robotArm.place()
            //robotArm.bringUp()
            robot.mecanumDriveStraightAlongZero(-0.5, -66, 0); //x: 48 30 y: -34 -34
            robot.mecanumDriveStraightAlongZero(-0.5, -24, 0); //x: 48 30 y: -34 -34
            //robot.pickupBlock()
            robot.mecanumDriveStraightAlongZero(0.5, 24, 0); //x: 48 30 y: -34 -34
            robot.mecanumDriveStraightAlongZero(0.5, 66, 0); //x: 48 30 y: -34 -34
            //robot.placeBlock()
        }
        else {
            robot.mecanumDriveStraightAlongZero(-0.5, -8, 0); //x: -26 -44 y: -34 -34
            //colorSensor.getColor()
            if (color.equals("black")) {
                //robotArm.bringUp()
                //robotArm.bringDown()
                //robotArm.bringUp()
                robot.mecanumDriveStraightAlongZero(0.5, 74, 0); //x: 48 30 y: -34 -34
                //robotArm.bringDown()
                //robotArm.place()
                //robotArm.bringUp()
                robot.mecanumDriveStraightAlongZero(-0.5, -74, 0); //x: -26 -44 y: -34 -34
                robot.mecanumDriveStraightAlongZero(-0.5, -24, 0); //x: -50 -68 y: -34 -34
                //robot.pickupBlock()
                robot.mecanumDriveStraightAlongZero(0.5, 24, 0); //x: -26 -44 y: -34 -34
                robot.mecanumDriveStraightAlongZero(0.5, 74, 0); //x: 48 30 y: -34 -34
                //robot.placeBlock()
            }
            else {
                robot.mecanumDriveStraightAlongZero(-0.5, -8, 0); //x: -34 -52 y: -34 -34
                //robotArm.bringUp()
                //robotArm.bringDown()
                //robotArm.bringUp()
                robot.mecanumDriveStraightAlongZero(0.5, 82, 0); //x: 48 30 y: -34 -34
                //robotArm.bringUp()
                //robotArm.bringDown()
                //robotArm.bringUp()
                robot.mecanumDriveStraightAlongZero(-0.5, -82, 0); //x: -34 -52 y: -34 -34
                robot.mecanumDriveStraightAlongZero(-0.5, -24, 0); //x: -58 -76 y: -34 -34
                //robot.pickupBlock()
                robot.mecanumDriveStraightAlongZero(0.5, 24, 0); //x: -34 -52 y: -34 -34
                robot.mecanumDriveStraightAlongZero(0.5, 82, 0); //x: 48 30 y: -34 -34
                //robot.placeBlock()
            }
        }
        robot.turnRobot(0.4,90); //x: 48 30 y: -34 -34
        robot.mecanumDriveStraightAlongZero(-0.5,-10,90); //x: 48 30 y: -24 -24

        robot.mecanumStrafe(0.5, 10,MovementDirection.RIGHT, 90); //58 40 y: -24 -24
        foundation.clampFoundation();
        robot.mecanumStrafe(0.5, 10,MovementDirection.LEFT, 90); //48 30 y: -24 -24
        robot.mecanumDriveStraightAlongZero(0.5,25,90); //x: 48 30 y: -49 -49
        robot.turnRobot(0.4, 180); //turn 90 counter-clockwise
        robot.mecanumDriveStraightAlongZero(-0.8,-25,180);
        foundation.releaseFoundation();
        robot.mecanumDriveStraightAlongZero(0.5,40,180);
        robot.mecanumStrafe(0.5, 10,MovementDirection.LEFT, 180); //48 30 y: -24 -24
    }
}