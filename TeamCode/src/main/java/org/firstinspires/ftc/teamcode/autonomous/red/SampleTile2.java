package org.firstinspires.ftc.teamcode.autonomous.red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.RobotAlliance;
import org.firstinspires.ftc.teamcode.autonomous.base_classes.SampleTile2Base;

@Autonomous(name = "Red Sample (Tile 2)", group = "AAR")
public class SampleTile2 extends SampleTile2Base {
    @Override
    protected RobotAlliance getCurrentAlliance() {
        return RobotAlliance.RED;
    }
}
