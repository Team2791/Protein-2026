package frc.robot.data;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;

public record ClosedLoopData(
    double target,
    ControlType control,
    boolean atTarget
) {
    public static ClosedLoopData read(SparkClosedLoopController ctl) {
        return new ClosedLoopData(
            ctl.getSetpoint(),
            ctl.getControlType(),
            ctl.isAtSetpoint()
        );
    }

    public static ClosedLoopData empty() {
        return new ClosedLoopData(0.0, ControlType.kDutyCycle, false);
    }
}
