package frc.robot.util;

import frc.robot.constants.RuntimeConstants;
import java.util.function.Supplier;

public final class AdvantageUtil {

    private AdvantageUtil() {}

    public static <T> T match(T real, T replay) {
        return switch (RuntimeConstants.kCurrentMode) {
            case REAL -> real;
            case REPLAY, SIM -> replay;
        };
    }

    public static <T> T match(Supplier<T> real, Supplier<T> replay) {
        return AdvantageUtil.<Supplier<T>>match(real, replay).get();
    }
}
