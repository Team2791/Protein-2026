package frc.robot.auto;

import static frc.robot.constants.RobotConstants.DriveBase.kBumperRadius;

import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.util.Vec2;
import java.io.File;
import java.util.*;

class FieldSize {

    public double x;
    public double y;
}

class NavgridSchema {

    @JsonProperty("field_size")
    public FieldSize field_size;

    @JsonProperty("nodeSizeMeters")
    public double node_size_meters;

    public boolean[][] grid;
}

public class AStar {

    public static final NavgridSchema ng;

    static {
        ObjectMapper mapper = new ObjectMapper();
        try {
            ng = mapper.readValue(
                new File(Filesystem.getDeployDirectory() + "/navgrid.json"),
                NavgridSchema.class
            );
        } catch (Exception e) {
            throw new RuntimeException(e);
        }

        // Run this once after ng is loaded
        int radiusCells = (int) Math.floor(kBumperRadius / ng.node_size_meters);
        int rows = ng.grid.length;
        int cols = ng.grid[0].length;
        boolean[][] inflated = new boolean[rows][cols];

        for (int y = 0; y < rows; y++) {
            for (int x = 0; x < cols; x++) {
                if (!ng.grid[y][x]) continue; // not an obstacle, skip

                // Mark all cells within radius of this obstacle as blocked
                for (int dy = -radiusCells; dy <= radiusCells; dy++) {
                    for (int dx = -radiusCells; dx <= radiusCells; dx++) {
                        // if (
                        //     dx * dx + dy * dy > radiusCells * radiusCells
                        // ) continue; // circle, not square
                        int nx = x + dx,
                            ny = y + dy;
                        if (nx >= 0 && nx < cols && ny >= 0 && ny < rows) {
                            inflated[ny][nx] = true;
                        }
                    }
                }
            }
        }

        ng.grid = inflated;
    }

    private record Node(
        int x,
        int y,
        double g,
        double f,
        Node parent
    ) implements Comparable<Node> {
        public int compareTo(Node other) {
            return Double.compare(this.f, other.f);
        }
    }

    private static int rows() {
        return ng.grid.length;
    }

    private static int cols() {
        return ng.grid[0].length;
    }

    private static boolean isWalkable(int x, int y) {
        if (x < 0 || y < 0 || x >= cols() || y >= rows()) return false;
        return !ng.grid[y][x];
    }

    private static double heuristic(int x, int y, int ex, int ey) {
        return ng.node_size_meters * (Math.abs(x - ex) + Math.abs(y - ey));
    }

    public static Vec2[] path(Vec2 s, Vec2 e) {
        int startX = (int) MathUtil.clamp(
            s.x / ng.node_size_meters,
            0,
            cols() - 1
        );
        int startY = (int) MathUtil.clamp(
            s.y / ng.node_size_meters,
            0,
            rows() - 1
        );
        int endX = (int) MathUtil.clamp(
            e.x / ng.node_size_meters,
            0,
            cols() - 1
        );
        int endY = (int) MathUtil.clamp(
            e.y / ng.node_size_meters,
            0,
            rows() - 1
        );

        if (!isWalkable(startX, startY) || !isWalkable(endX, endY)) {
            return new Vec2[0];
        }

        PriorityQueue<Node> open = new PriorityQueue<>();
        double[][] bestG = new double[rows()][cols()];
        for (double[] row : bestG) Arrays.fill(row, Double.MAX_VALUE);

        open.add(
            new Node(
                startX,
                startY,
                0.0,
                heuristic(startX, startY, endX, endY),
                null
            )
        );
        bestG[startY][startX] = 0.0;

        int[] dx = { 1, -1, 0, 0 };
        int[] dy = { 0, 0, 1, -1 };
        double[] moveCost = { 1, 1, 1, 1 };

        while (!open.isEmpty()) {
            Node cur = open.poll();
            if (cur.g() > bestG[cur.y()][cur.x()]) continue;

            if (cur.x() == endX && cur.y() == endY) {
                return reconstructPath(cur);
            }

            for (int i = 0; i < 4; i++) {
                int nx = cur.x() + dx[i];
                int ny = cur.y() + dy[i];
                if (!isWalkable(nx, ny)) continue;
                if (
                    i >= 4 &&
                    (!isWalkable(cur.x() + dx[i], cur.y()) ||
                        !isWalkable(cur.x(), cur.y() + dy[i]))
                ) continue;

                double ng2 = cur.g() + moveCost[i] * ng.node_size_meters;
                if (ng2 < bestG[ny][nx]) {
                    bestG[ny][nx] = ng2;
                    open.add(
                        new Node(
                            nx,
                            ny,
                            ng2,
                            ng2 + heuristic(nx, ny, endX, endY),
                            cur
                        )
                    );
                }
            }
        }

        return new Vec2[0];
    }

    private static Vec2[] reconstructPath(Node goal) {
        List<Vec2> pts = new ArrayList<>();
        for (Node n = goal; n != null; n = n.parent()) {
            // Change: Vec2 constructor directly builds the waypoint
            pts.add(
                new Vec2(
                    (n.x() + 0.5) * ng.node_size_meters,
                    (n.y() + 0.5) * ng.node_size_meters
                )
            );
        }
        Collections.reverse(pts);
        return pts.toArray(new Vec2[0]);
    }

    public static boolean raycast(Vec2 from, Vec2 to) {
        int x0 = (int) (from.x / ng.node_size_meters);
        int y0 = (int) (from.y / ng.node_size_meters);
        int x1 = (int) (to.x / ng.node_size_meters);
        int y1 = (int) (to.y / ng.node_size_meters);

        int dx = Math.abs(x1 - x0);
        int sx = x0 < x1 ? 1 : -1;
        int dy = -Math.abs(y1 - y0);
        int sy = y0 < y1 ? 1 : -1;
        int err = dx + dy;

        while (true) {
            if (!isWalkable(x0, y0)) return false;
            if (x0 == x1 && y0 == y1) break;

            int e2 = 2 * err;
            boolean stepX = e2 >= dy;
            boolean stepY = e2 <= dx;

            // On a diagonal step, also check the two axis-aligned neighbors
            // to prevent squeezing through diagonal obstacle corners
            if (stepX && stepY) {
                if (
                    !isWalkable(x0 + sx, y0) || !isWalkable(x0, y0 + sy)
                ) return false;
            }

            if (stepX) {
                err += dy;
                x0 += sx;
            }
            if (stepY) {
                err += dx;
                y0 += sy;
            }
        }
        return true;
    }

    public static List<Pose2d> debug() {
        int rows = ng.grid.length;
        int cols = ng.grid[0].length;
        List<Pose2d> pts = new ArrayList<>();
        for (int y = 0; y < rows; y++) {
            for (int x = 0; x < cols; x++) {
                if (ng.grid[y][x]) {
                    pts.add(
                        new Pose2d(
                            (x + 0.5) * ng.node_size_meters,
                            (y + 0.5) * ng.node_size_meters,
                            new Rotation2d()
                        )
                    );
                }
            }
        }

        return pts;
    }
}
