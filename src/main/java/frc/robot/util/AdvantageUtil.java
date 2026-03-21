package frc.robot.util;

import frc.robot.constants.RuntimeConstants;
import java.util.function.Supplier;

public final class AdvantageUtil {

    private AdvantageUtil() {}

    public static <T> T match(T real, T replay) {
        return match(real, replay, replay);
    }

    public static <T> T match(T real, T sim, T replay) {
        return switch (RuntimeConstants.kCurrentMode) {
            case REAL -> real;
            case SIM -> sim;
            case REPLAY -> replay;
        };
    }

    public static <T> T match(Supplier<T> real, Supplier<T> replay) {
        return match(real, replay, replay);
    }

    public static <T> T match(
        Supplier<T> real,
        Supplier<T> sim,
        Supplier<T> replay
    ) {
        return AdvantageUtil.<Supplier<T>>match(real, sim, replay).get();
    }
}
