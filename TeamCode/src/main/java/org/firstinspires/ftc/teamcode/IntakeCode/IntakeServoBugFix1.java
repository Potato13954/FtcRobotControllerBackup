package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name="FTC Starter Kit Example Robot (INTO THE DEEP)", group="Robot")
//@Disabled
public class IntakeServoBugFix1 extends LinearOpMode {

    /* Declare OpMode members. */
    public CRServo intake = null; // the active intake servo

    final double INTAKE_COLLECT = -1.0; // Power for collecting
    final double INTAKE_OFF = 0.0; // Power for off
    final double INTAKE_DEPOSIT = 0.5; // Power for depositing

    @Override
    public void runOpMode() {

        intake = hardwareMap.get(CRServo.class, "intake");
        intake.setPower(INTAKE_OFF);

        telemetry.addLine("Robot Ready.");
        telemetry.update();

        waitForStart();

        /* Run until the driver presses stop */
        while (opModeIsActive()) {
            if (gamepad1.a) {
                // A is held, run the intake
                intake.setPower(INTAKE_COLLECT);
            } else if (gamepad1.x) {
                // X is pressed, turn off the intake
                intake.setPower(INTAKE_OFF);
            } else if (gamepad1.b) {
                // B is pressed, run the intake in deposit mode
                intake.setPower(INTAKE_DEPOSIT);
            } else {
                // When no button is pressed, maintain the current power to prevent reversing
                intake.setPower(intake.getPower());
            }
        }
    }
}
