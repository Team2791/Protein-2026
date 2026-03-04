// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.util;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class SparkUtil {

    /**
     * Set to {@code true} by any utility method in this class when a Spark returns an error.
     * Can be checked after a series of operations to see if any failed.
     */
    public static boolean sparkStickyFault = false;

    /**
     * Reads a value from a Spark and passes it to a consumer only if the Spark reported no error.
     * Sets {@link #sparkStickyFault} if an error is detected.
     *
     * @param spark    The Spark motor controller to check
     * @param supplier The value supplier (e.g. encoder position)
     * @param consumer The consumer to call with the valid value
     */
    public static void ifOk(
        SparkBase spark,
        DoubleSupplier supplier,
        DoubleConsumer consumer
    ) {
        double value = supplier.getAsDouble();
        if (spark.getLastError() == REVLibError.kOk) {
            consumer.accept(value);
        } else {
            sparkStickyFault = true;
        }
    }

    /**
     * Reads multiple values from a Spark and passes them to a consumer only if all reads succeeded.
     * Sets {@link #sparkStickyFault} and returns early on the first error.
     *
     * @param spark     The Spark motor controller to check
     * @param suppliers Array of value suppliers to read
     * @param consumer  The consumer to call with the array of valid values
     */
    public static void ifOk(
        SparkBase spark,
        DoubleSupplier[] suppliers,
        Consumer<double[]> consumer
    ) {
        double[] values = new double[suppliers.length];
        for (int i = 0; i < suppliers.length; i++) {
            values[i] = suppliers[i].getAsDouble();
            if (spark.getLastError() != REVLibError.kOk) {
                sparkStickyFault = true;
                return;
            }
        }
        consumer.accept(values);
    }

    /**
     * Retries a Spark command up to {@code maxAttempts} times until it returns {@link REVLibError#kOk}.
     * Sets {@link #sparkStickyFault} on each failed attempt.
     *
     * @param spark       The Spark motor controller
     * @param maxAttempts Maximum number of attempts before giving up
     * @param command     The command to execute (e.g. a configuration call)
     */
    public static void tryUntilOk(
        SparkBase spark,
        int maxAttempts,
        Supplier<REVLibError> command
    ) {
        for (int i = 0; i < maxAttempts; i++) {
            var error = command.get();
            if (error == REVLibError.kOk) {
                break;
            } else {
                sparkStickyFault = true;
            }
        }
    }
}
