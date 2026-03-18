package frc.robot.alerter;

import frc.robot.util.Elastic;
import frc.robot.util.Elastic.Notification.NotificationLevel;
import java.util.function.Function;

/**
 * Internal record representing a monitored device and its error state.
 *
 * @param <T> The device type
 * @param <E> The error type for this device
 * @param device The device instance being monitored
 * @param name Human-readable name of the device
 * @param error Function to extract the current error state from the device
 * @param serialize Function to convert an error to a human-readable description
 * @param signaled List of errors that have already been reported (to avoid duplicates)
 */
record Device<T, E>(
    T device,
    String name,
    Function<T, E> error,
    Function<E, String> serialize,
    Cell<E> previous
) {
    static class Cell<T> {

        T previous;

        Cell(T previous) {
            this.previous = previous;
        }

        public boolean equals(Object other) {
            return previous.equals(other);
        }

        public void set(T previous) {
            this.previous = previous;
        }
    }

    Device(
        T deivce,
        String name,
        Function<T, E> error,
        Function<E, String> serialize,
        E skip
    ) {
        this(deivce, name, error, serialize, new Cell<E>(skip));
    }

    Device(
        T deivce,
        String name,
        Function<T, E> error,
        Function<E, String> serialize
    ) {
        this(deivce, name, error, serialize, new Cell<E>(null));
    }

    /**
     * Checks the device for a new error and alerts if one is found.
     *
     * Only reports each unique error once - subsequent occurrences are ignored.
     */
    void alert() {
        E err = error.apply(device);
        if (err == null || previous.equals(err)) return;

        // Send notification to Elastic dashboard
        Elastic.sendNotification(
            new Elastic.Notification(
                NotificationLevel.ERROR,
                "Device failed",
                String.format("%s reported: %s", name, serialize.apply(err))
            )
        );

        previous.set(err);
    }
}
