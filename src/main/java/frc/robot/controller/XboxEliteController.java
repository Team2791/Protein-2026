package frc.robot.controller;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Extended Xbox controller with support for the four back paddles on the Xbox Elite Series 2.
 *
 * <p>
 * Paddle inputs are read via {@link KBNT} (Keyboard-over-NetworkTables), which requires the
 * KBNT companion app to be running on the driver station and each paddle mapped to a key.
 *
 * <p>
 * The paddles are ordered [Bottom-Left, Top-Left, Top-Right, Bottom-Right] to match the
 * left-to-right order shown in the Xbox Accessories configuration app.
 */
public class XboxEliteController extends CommandXboxController {

    /**
     * Paddle triggers in the following order: [BL, TL, TR, BR].
     * <br> They will read left-to-right in the Xbox Accessories app.
     */
    final Trigger[] paddles = new Trigger[4];

    /**
     * Creates an XboxEliteController instance with paddle triggers mapped to the specified keys.
     *
     * @param port The USB port number for the controller (e.g., 0 for the first controller).
     * @param paddleKeys The keys assigned to the paddles in order [BL, TL, TR, BR]. For example, "team" would assign 't' to BL, 'e' to TL, and so on.
     */
    public XboxEliteController(int port, String paddleKeys) {
        super(port);
        KBNT kbnt = KBNT.getInstance();
        for (int i = 0; i < 4; i++) {
            paddles[i] = kbnt.trigger(paddleKeys.charAt(i));
        }
    }

    /**
     * Bottom-left paddle (P1)
     *
     * @return Trigger for the bottom-left back paddle (P1).
     */
    public Trigger paddleBL() {
        return paddles[0];
    }

    /**
     * Top-left paddle (P3)
     *
     * @return Trigger for the top-left back paddle (P3).
     */
    public Trigger paddleTL() {
        return paddles[1];
    }

    /**
     * Top-right paddle (P4)
     *
     * @return Trigger for the top-right back paddle (P4).
     */
    public Trigger paddleTR() {
        return paddles[2];
    }

    /**
     * Bottom-right paddle (P2)
     *
     * @return Trigger for the bottom-right back paddle (P2).
     */
    public Trigger paddleBR() {
        return paddles[3];
    }
}
