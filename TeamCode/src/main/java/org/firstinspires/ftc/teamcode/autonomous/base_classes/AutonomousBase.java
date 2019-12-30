package org.firstinspires.ftc.teamcode.autonomous.base_classes;

import org.firstinspires.ftc.teamcode.autonomous.AutonomousMaster;
import org.firstinspires.ftc.teamcode.autonomous.RobotAlliance;

public abstract class AutonomousBase extends AutonomousMaster {
    protected RobotAlliance currentAlliance;

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        currentAlliance = getCurrentAlliance();
    }

    // Define this method in the alliance-specific implementations of this base class
    // For example, if implementing this class for usage on the red alliance, this should contain "return RobotAlliance.RED;"
    // This will be used runOpMode() to handle the slight differences in paths by alliance (eg turning CCW instead of CW)
    protected abstract RobotAlliance getCurrentAlliance();
}
