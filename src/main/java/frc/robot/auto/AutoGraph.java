package frc.robot.auto;

import frc.robot.auto.generated.ChoreoTraj;
import java.util.EnumMap;
import java.util.Map;

/**
 * A graph-based representation of autonomous navigation paths for the robot.
 *
 * <p>This class builds a transition map that connects autonomous nodes (positions/states)
 * through Choreo trajectories. It automatically parses trajectory names to determine
 * the source and destination nodes for each path.
 *
 * <p>The class expects trajectory names to follow the format "from_to" where "from" and "to"
 * are valid AutoNode enum values (lowercased). Trajectories with names starting with "seq_" or not
 * containing exactly one underscore are ignored.
 *
 * <p>The resulting TRANSITIONS map provides O(1) lookup for finding trajectories
 * between any two connected autonomous nodes, enabling efficient path planning
 * and autonomous routine construction.
 */
public class AutoGraph {

    public static final Map<AutoNode, Map<AutoNode, ChoreoTraj>> TRANSITIONS =
        calculate();

    static Map<AutoNode, Map<AutoNode, ChoreoTraj>> calculate() {
        Map<AutoNode, Map<AutoNode, ChoreoTraj>> out = new EnumMap<>(
            AutoNode.class
        );

        for (ChoreoTraj traj : ChoreoTraj.ALL_TRAJECTORIES.values()) {
            String name = traj.name();
            if (name.startsWith("seq_")) continue;
            if (!name.contains("_")) continue;

            String[] parts = name.split("_");
            if (parts.length != 2) continue;

            AutoNode from;
            AutoNode to;

            try {
                from = AutoNode.valueOf(parts[0].toUpperCase());
                to = AutoNode.valueOf(parts[1].toUpperCase());
            } catch (IllegalArgumentException e) {
                continue;
            }

            out.putIfAbsent(from, new EnumMap<>(AutoNode.class));
            out.get(from).put(to, traj);
        }

        return out;
    }
}
