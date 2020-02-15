package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

public class DriveTrain {
    private DcMotor  rightFrontMotor, leftFrontMotor, rightBackMotor, leftBackMotor = null;
    private final double RADIUS = 2;
    private final double TICKS_PER_REVOLUTION = 537.6;
    private final double COUNTS_PER_INCH = TICKS_PER_REVOLUTION/(2 * Math.PI * RADIUS);
    private final double TOLERANCE_FOR_ROTATING_ROBOT_POSITION = 1;
    private HardwareMap hwMap           =  null;
    private BNO055IMU imu;
    private Orientation angles;

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        rightFrontMotor  = hwMap.get(DcMotor.class, "rfm");
        leftFrontMotor  = hwMap.get(DcMotor.class, "lfm");
        rightBackMotor = hwMap.get(DcMotor.class, "rbm");
        leftBackMotor  = hwMap.get(DcMotor.class, "lbm");

        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);

        setPower(0,0,0,0);

        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;

        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }

    public void setPower(double rightFront, double leftFront, double rightBack, double leftBack) {
        rightFrontMotor.setPower(rightFront);
        leftFrontMotor.setPower(leftFront);
        rightBackMotor.setPower(rightBack);
        leftBackMotor.setPower(leftBack);
    }

    private void setTargetPositions(double inches) {
        int newRightFrontTarget = rightFrontMotor.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
        int newLeftFrontTarget = leftFrontMotor.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
        int newRightBackTarget = rightBackMotor.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
        int newLeftBackTarget = leftBackMotor.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
        rightFrontMotor.setTargetPosition(newRightFrontTarget);
        leftFrontMotor.setTargetPosition(newLeftFrontTarget);
        rightBackMotor.setTargetPosition(newLeftBackTarget);
        leftBackMotor.setTargetPosition(newLeftFrontTarget);
    }

    private void activateRunToPositionMode() {
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void activateRunUsingEncoderMode() {
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void shutDown() {
        setPower(0,0,0,0);
    }

    public void mecanumDriveStraightAlongZero(double speed, double inches, double relativeAngle) {
        setTargetPositions(inches);
        activateRunToPositionMode();

        double leftSpeed = speed; double rightSpeed = speed;
        setPower(speed, speed, speed, speed);

        while ((rightFrontMotor.isBusy() && leftFrontMotor.isBusy())) {

            double error = getError(relativeAngle);
            double steeringError = getSteeringError(error, 0.1);

            leftSpeed = speed-steeringError;
            rightSpeed = speed+steeringError;

            double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));

            if (max > 1) {
                leftSpeed /= (max);
                rightSpeed /= (max);
            }

            setPower(rightSpeed, leftSpeed, rightSpeed, leftSpeed);
        }

        activateRunUsingEncoderMode();
        setPower(0,0,0,0);
    }

    public void mecanumStrafe(double speed, double inches, MovementDirection dir, double relativeAngle) {
        setTargetPositions(inches);

        double topLeftSpeed = speed; double topRightSpeed = speed;
        double bottomLeftSpeed = speed; double bottomRightSpeed = speed;


        if (dir == MovementDirection.RIGHT) {
                bottomLeftSpeed *= -1;
                topRightSpeed *= -1;
        } else if (dir == MovementDirection.LEFT) {
                bottomRightSpeed *= -1;
                topLeftSpeed *= -1;
        }

        activateRunToPositionMode();
        setPower(topLeftSpeed, topRightSpeed, bottomLeftSpeed, bottomRightSpeed);

        while ((rightFrontMotor.isBusy() && leftFrontMotor.isBusy())) {
            double error = getError(relativeAngle);
            double steeringError = getSteeringError(error, 0.1);

            topLeftSpeed = topLeftSpeed-steeringError;
            topRightSpeed = topRightSpeed+steeringError;
            bottomLeftSpeed = bottomLeftSpeed-steeringError;
            bottomRightSpeed = bottomRightSpeed+steeringError;

            double max = Math.max(Math.abs(topLeftSpeed), Math.abs(topRightSpeed));

            if (max > 1) {
                topRightSpeed /= max;
                bottomRightSpeed /= max;
                topLeftSpeed /= max;
                bottomLeftSpeed /= max;
            }

            setPower(topLeftSpeed, topRightSpeed, bottomRightSpeed, bottomLeftSpeed);
        }

        activateRunUsingEncoderMode();
        setPower(0,0,0,0);
    }

    public void turnRobot(double speed, double degrees) { //turns counterclockwise
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double target = (degrees + angles.firstAngle);
        double robotError = getError(target);

        while (Math.abs(robotError) < TOLERANCE_FOR_ROTATING_ROBOT_POSITION) {
            robotError = getError(target);
            double tempSpeed = speed;
            tempSpeed *= getSteeringError(robotError, 0.1);
            tempSpeed = Math.max(tempSpeed, 0.8);
            setPower(tempSpeed, -tempSpeed, tempSpeed, -tempSpeed);
        }
        setPower(0,0,0,0);

    }

    public double getError(double targetAngle) { // calculate error in -179 to +180 range
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double robotError = targetAngle - angles.firstAngle;
        while (robotError > 180)  {
            robotError -= 360;
        }
        while (robotError <= -180) {
            robotError += 360;
        }
        return robotError;
    }

    public double getSteeringError(double error, double P_COEFF) {
        return error * P_COEFF;
    }

}
