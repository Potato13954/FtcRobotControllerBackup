

package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name = "IntakeServoTest", group = "Autonomous")
public class AutoParking extends LinearOpMode {

    double power = 0.5;

    private DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive;
    private CRServo intake;
    private DcMotorEx armMotor;

    @Override
    public void runOpMode() {
        // Initialize motors
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        intake = hardwareMap.get(CRServo.class, "intake");
        armMotor = hardwareMap.get(DcMotorEx.class, "left_arm");

        //reverse the motor to be backwards because of orientation
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        //wait until you hit start to go
        waitForStart();

        //put the code for the movement here

        // example: IntakeRotation(intakeOperations.INTAKE_IN, 0.5f, 1);
    }

    enum movement {
        FORWARD,
        BACKWARDS,
        LEFT,
        RIGHT
    }

    enum intakeOperations {
        INTAKE_IN,
        INTAKE_OUT
    }

    enum armOperations {
        ARM_FORWARD,
        ARM_BACKWARDS
    }

    private void move(movement movement, double power, int duration) {
        if(movement == AutoParking.movement.FORWARD)  {
            //set all motors to positive power to move forward
            leftFrontDrive.setPower(power);
            rightFrontDrive.setPower(power);
            leftBackDrive.setPower(power);
            rightBackDrive.setPower(power);

        } else if(movement == AutoParking.movement.BACKWARDS) {
            //set all motors to negative power to move backwards
            leftFrontDrive.setPower(-power);
            rightFrontDrive.setPower(-power);
            leftBackDrive.setPower(-power);
            rightBackDrive.setPower(-power);

        } else if(movement == AutoParking.movement.LEFT) {
            //set diagonal motors to negative to move left
            leftFrontDrive.setPower(-power);
            rightFrontDrive.setPower(power);
            leftBackDrive.setPower(power);
            rightBackDrive.setPower(-power);

        } else if(movement == AutoParking.movement.RIGHT) {
            //set diagonal motors to positive to move right
            leftFrontDrive.setPower(power);
            rightFrontDrive.setPower(-power);
            leftBackDrive.setPower(-power);
            rightBackDrive.setPower(power);
        }
        //sleep for duration so not instant
        sleep(duration);
        //stop the motors
        stopMotors();
    }

    private void IntakeRotation(intakeOperations operation, double power, int duration) {
        if(operation == intakeOperations.INTAKE_IN) {
            //move the intake in
            intake.setPower(power);
        } else if(operation == intakeOperations.INTAKE_OUT) {
            //move the intake out
            intake.setPower(-power);
        }
        //sleep for duration so not instant
        sleep(duration);
        //stop the motors
        stopIntake();
    }
    private void ArmRotation(armOperations operation, double power, int duration) {
        //if it's operation is = to forwards, make it positive
        if(operation == armOperations.ARM_FORWARD) {
            armMotor.setPower(power);
        } else if(operation == armOperations.ARM_BACKWARDS) {
            //if = to backwards, back it negative
            armMotor.setPower(-power);
        }
        //sleep for duration so not instant
        sleep(duration);
        //stop the motors
        stopArm();
    }

    private void stopMotors() {
        //stop all motors
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }
    private void stopIntake() {
        //stop the intake
        intake.setPower(0);
    }
    private void stopArm() {
        //stop the arm
        armMotor.setPower(0);
    }
}
