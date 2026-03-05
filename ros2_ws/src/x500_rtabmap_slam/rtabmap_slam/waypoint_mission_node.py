#!/usr/bin/env python3
"""Map-driven mission node for autonomous maze navigation.

This node does not use hardcoded waypoints. It accepts one final goal and
continuously publishes intermediate subgoals to /goal_pose using:
1) A* over the current occupancy grid map (/map)
2) Frontier fallback if the final goal is not yet connected in known free space
For goal points outside current map bounds, it prioritizes frontier expansion.
"""

from collections import deque
from heapq import heappop, heappush
import math
from typing import Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from tf2_ros import Buffer, TransformException, TransformListener

Cell = Tuple[int, int]


class WaypointMissionNode(Node):
    def __init__(self):
        super().__init__("waypoint_mission")

        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("final_goal_topic", "/maze_final_goal")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("map_frame_id", "map")
        self.declare_parameter("base_frame_id", "base_link")
        self.declare_parameter("goal_z", 0.0)
        self.declare_parameter("publish_period_sec", 0.5)
        self.declare_parameter("start_delay_sec", 0.0)
        self.declare_parameter("final_goal_xy", [])
        self.declare_parameter("final_goal_reach_tolerance", 0.60)
        self.declare_parameter("subgoal_lookahead_m", 1.8)
        self.declare_parameter("retarget_min_separation_m", 0.35)
        self.declare_parameter("occupied_threshold", 60)
        self.declare_parameter("unknown_is_blocked", True)
        self.declare_parameter("inflation_radius_m", 0.12)
        self.declare_parameter("start_exempt_radius_m", 0.20)
        self.declare_parameter("max_goal_search_radius_m", 2.5)
        self.declare_parameter("frontier_min_distance_m", 0.8)
        self.declare_parameter("frontier_distance_weight", 0.15)

        self.goal_topic = str(self.get_parameter("goal_topic").value)
        self.final_goal_topic = str(self.get_parameter("final_goal_topic").value)
        self.map_topic = str(self.get_parameter("map_topic").value)
        self.map_frame_id = str(self.get_parameter("map_frame_id").value)
        self.base_frame_id = str(self.get_parameter("base_frame_id").value)
        self.goal_z = float(self.get_parameter("goal_z").value)
        self.publish_period = float(self.get_parameter("publish_period_sec").value)
        self.start_delay = float(self.get_parameter("start_delay_sec").value)
        self.final_goal_reach_tolerance = float(
            self.get_parameter("final_goal_reach_tolerance").value
        )
        self.subgoal_lookahead_m = float(self.get_parameter("subgoal_lookahead_m").value)
        self.retarget_min_separation_m = float(
            self.get_parameter("retarget_min_separation_m").value
        )
        self.occupied_threshold = int(self.get_parameter("occupied_threshold").value)
        self.unknown_is_blocked = bool(self.get_parameter("unknown_is_blocked").value)
        self.inflation_radius_m = float(self.get_parameter("inflation_radius_m").value)
        self.start_exempt_radius_m = float(self.get_parameter("start_exempt_radius_m").value)
        self.max_goal_search_radius_m = float(
            self.get_parameter("max_goal_search_radius_m").value
        )
        self.frontier_min_distance_m = float(
            self.get_parameter("frontier_min_distance_m").value
        )
        self.frontier_distance_weight = float(
            self.get_parameter("frontier_distance_weight").value
        )

        self.goal_pub = self.create_publisher(PoseStamped, self.goal_topic, 10)
        self.final_goal_sub = self.create_subscription(
            PoseStamped, self.final_goal_topic, self._on_final_goal, 10
        )
        # RTAB-Map occupancy often behaves like a latched topic; use transient-local
        # so a late-joining mission node still receives the latest map snapshot.
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid, self.map_topic, self._on_map, map_qos
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.start_time = self.get_clock().now()
        self.started = False
        self.last_status_log_ns = 0
        self.last_mode: Optional[str] = None

        self.map_msg: Optional[OccupancyGrid] = None
        self.active_subgoal_xy: Optional[Tuple[float, float]] = None
        self.final_goal_xy: Optional[Tuple[float, float]] = None
        self.final_goal_reached = False

        self._disk_offsets_cache: Dict[int, List[Tuple[int, int]]] = {}

        initial_goal = list(self.get_parameter("final_goal_xy").value)
        if len(initial_goal) == 2:
            self.final_goal_xy = (float(initial_goal[0]), float(initial_goal[1]))
            self.get_logger().info(
                "Initial final goal from parameter: (%.2f, %.2f)"
                % self.final_goal_xy
            )
        elif len(initial_goal) != 0:
            self.get_logger().warn(
                "Ignoring final_goal_xy: expected exactly 2 values [x, y], got %d"
                % len(initial_goal)
            )

        self.timer = self.create_timer(self.publish_period, self._on_timer)

        self.get_logger().info(
            "Map-driven mission ready: waiting for final goal on %s (publishing to %s)"
            % (self.final_goal_topic, self.goal_topic)
        )

    def _on_map(self, msg: OccupancyGrid) -> None:
        self.map_msg = msg

    def _on_final_goal(self, msg: PoseStamped) -> None:
        if msg.header.frame_id and msg.header.frame_id != self.map_frame_id:
            self.get_logger().warn(
                "Final goal frame is '%s' (expected '%s'); using coordinates as map frame."
                % (msg.header.frame_id, self.map_frame_id)
            )
        self.final_goal_xy = (float(msg.pose.position.x), float(msg.pose.position.y))
        self.active_subgoal_xy = None
        self.final_goal_reached = False
        self.last_mode = None
        self.get_logger().info(
            "Received new final goal: (%.2f, %.2f)" % self.final_goal_xy
        )

    def _lookup_pose_xy(self) -> Optional[Tuple[float, float]]:
        try:
            tf_msg = self.tf_buffer.lookup_transform(
                self.map_frame_id,
                self.base_frame_id,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05),
            )
            t = tf_msg.transform.translation
            return float(t.x), float(t.y)
        except TransformException:
            return None

    def _publish_goal(self, x: float, y: float) -> None:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame_id
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = self.goal_z
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0
        self.goal_pub.publish(msg)

    def _in_bounds(self, x: int, y: int, width: int, height: int) -> bool:
        return 0 <= x < width and 0 <= y < height

    def _cell_to_index(self, x: int, y: int, width: int) -> int:
        return y * width + x

    def _world_to_cell(
        self, x: float, y: float, origin_x: float, origin_y: float, resolution: float
    ) -> Cell:
        gx = int(math.floor((x - origin_x) / resolution))
        gy = int(math.floor((y - origin_y) / resolution))
        return (gx, gy)

    def _cell_to_world(
        self, gx: int, gy: int, origin_x: float, origin_y: float, resolution: float
    ) -> Tuple[float, float]:
        wx = origin_x + (gx + 0.5) * resolution
        wy = origin_y + (gy + 0.5) * resolution
        return (wx, wy)

    def _disk_offsets(self, radius_cells: int) -> List[Tuple[int, int]]:
        if radius_cells <= 0:
            return [(0, 0)]
        cached = self._disk_offsets_cache.get(radius_cells)
        if cached is not None:
            return cached
        offsets: List[Tuple[int, int]] = []
        rr = radius_cells * radius_cells
        for dy in range(-radius_cells, radius_cells + 1):
            for dx in range(-radius_cells, radius_cells + 1):
                if dx * dx + dy * dy <= rr:
                    offsets.append((dx, dy))
        self._disk_offsets_cache[radius_cells] = offsets
        return offsets

    def _build_blocked_grid(
        self,
        map_msg: OccupancyGrid,
        current_cell: Cell,
    ) -> bytearray:
        width = int(map_msg.info.width)
        height = int(map_msg.info.height)
        resolution = float(map_msg.info.resolution)
        raw = map_msg.data
        blocked = bytearray(width * height)

        for idx, occ in enumerate(raw):
            if occ >= self.occupied_threshold or (
                self.unknown_is_blocked and occ < 0
            ):
                blocked[idx] = 1

        inflation_cells = int(math.ceil(self.inflation_radius_m / resolution))
        if inflation_cells > 0:
            offsets = self._disk_offsets(inflation_cells)
            inflated = bytearray(blocked)
            occupied_indices = [i for i, val in enumerate(blocked) if val]
            for idx in occupied_indices:
                x = idx % width
                y = idx // width
                for dx, dy in offsets:
                    nx = x + dx
                    ny = y + dy
                    if self._in_bounds(nx, ny, width, height):
                        inflated[self._cell_to_index(nx, ny, width)] = 1
            blocked = inflated

        exempt_cells = int(math.ceil(self.start_exempt_radius_m / resolution))
        if exempt_cells > 0:
            offsets = self._disk_offsets(exempt_cells)
            cx, cy = current_cell
            for dx, dy in offsets:
                nx = cx + dx
                ny = cy + dy
                if self._in_bounds(nx, ny, width, height):
                    blocked[self._cell_to_index(nx, ny, width)] = 0

        return blocked

    def _nearest_free_cell(
        self,
        desired: Cell,
        blocked: bytearray,
        width: int,
        height: int,
        resolution: float,
    ) -> Optional[Cell]:
        dx, dy = desired
        if not self._in_bounds(dx, dy, width, height):
            return None

        max_steps = max(1, int(math.ceil(self.max_goal_search_radius_m / resolution)))
        start_idx = self._cell_to_index(dx, dy, width)
        if blocked[start_idx] == 0:
            return desired

        q = deque([(dx, dy, 0)])
        seen = bytearray(width * height)
        seen[start_idx] = 1

        while q:
            x, y, d = q.popleft()
            if d > max_steps:
                continue
            idx = self._cell_to_index(x, y, width)
            if blocked[idx] == 0:
                return (x, y)
            for nx, ny in ((x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1)):
                if not self._in_bounds(nx, ny, width, height):
                    continue
                nidx = self._cell_to_index(nx, ny, width)
                if seen[nidx]:
                    continue
                seen[nidx] = 1
                q.append((nx, ny, d + 1))
        return None

    def _a_star(
        self,
        start: Cell,
        goal: Cell,
        blocked: bytearray,
        width: int,
        height: int,
    ) -> Optional[List[Cell]]:
        sidx = self._cell_to_index(start[0], start[1], width)
        gidx = self._cell_to_index(goal[0], goal[1], width)
        if blocked[sidx] or blocked[gidx]:
            return None

        neighbors = [
            (-1, 0, 1.0),
            (1, 0, 1.0),
            (0, -1, 1.0),
            (0, 1, 1.0),
            (-1, -1, math.sqrt(2.0)),
            (-1, 1, math.sqrt(2.0)),
            (1, -1, math.sqrt(2.0)),
            (1, 1, math.sqrt(2.0)),
        ]

        open_heap: List[Tuple[float, float, int, int]] = []
        came_from: Dict[Cell, Cell] = {}
        g_score: Dict[Cell, float] = {start: 0.0}
        closed = set()

        def heuristic(ax: int, ay: int, bx: int, by: int) -> float:
            return math.hypot(bx - ax, by - ay)

        heappush(
            open_heap,
            (
                heuristic(start[0], start[1], goal[0], goal[1]),
                0.0,
                start[0],
                start[1],
            ),
        )

        while open_heap:
            _, gcost, cx, cy = heappop(open_heap)
            current = (cx, cy)
            if current in closed:
                continue
            if current == goal:
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                return path
            closed.add(current)

            for dx, dy, step_cost in neighbors:
                nx = cx + dx
                ny = cy + dy
                if not self._in_bounds(nx, ny, width, height):
                    continue
                nidx = self._cell_to_index(nx, ny, width)
                if blocked[nidx]:
                    continue
                tentative = gcost + step_cost
                ncell = (nx, ny)
                if tentative >= g_score.get(ncell, float("inf")):
                    continue
                g_score[ncell] = tentative
                came_from[ncell] = current
                fcost = tentative + heuristic(nx, ny, goal[0], goal[1])
                heappush(open_heap, (fcost, tentative, nx, ny))
        return None

    def _select_path_lookahead_cell(
        self, path: List[Cell], resolution: float
    ) -> Cell:
        if len(path) <= 1:
            return path[-1]
        lookahead_cells = max(1e-6, self.subgoal_lookahead_m / resolution)
        dist_cells = 0.0
        for i in range(1, len(path)):
            x0, y0 = path[i - 1]
            x1, y1 = path[i]
            dist_cells += math.hypot(x1 - x0, y1 - y0)
            if dist_cells >= lookahead_cells:
                return path[i]
        return path[-1]

    def _has_unknown_neighbor(
        self,
        x: int,
        y: int,
        raw: List[int],
        width: int,
        height: int,
    ) -> bool:
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                nx = x + dx
                ny = y + dy
                if not self._in_bounds(nx, ny, width, height):
                    continue
                if raw[self._cell_to_index(nx, ny, width)] < 0:
                    return True
        return False

    def _best_frontier_cell(
        self,
        start: Cell,
        blocked: bytearray,
        map_msg: OccupancyGrid,
        current_xy: Tuple[float, float],
        final_goal_xy: Tuple[float, float],
        require_unknown_neighbor: bool = True,
        min_distance_m_override: Optional[float] = None,
    ) -> Optional[Cell]:
        width = int(map_msg.info.width)
        height = int(map_msg.info.height)
        resolution = float(map_msg.info.resolution)
        origin_x = float(map_msg.info.origin.position.x)
        origin_y = float(map_msg.info.origin.position.y)
        raw = map_msg.data

        sidx = self._cell_to_index(start[0], start[1], width)
        if blocked[sidx]:
            return None

        reachable = bytearray(width * height)
        q = deque([start])
        reachable[sidx] = 1

        while q:
            x, y = q.popleft()
            for nx, ny in ((x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1)):
                if not self._in_bounds(nx, ny, width, height):
                    continue
                nidx = self._cell_to_index(nx, ny, width)
                if reachable[nidx] or blocked[nidx]:
                    continue
                reachable[nidx] = 1
                q.append((nx, ny))

        best_cell: Optional[Cell] = None
        best_score = float("inf")

        min_distance_m = (
            self.frontier_min_distance_m
            if min_distance_m_override is None
            else float(min_distance_m_override)
        )

        for idx, is_reachable in enumerate(reachable):
            if not is_reachable:
                continue
            if blocked[idx]:
                continue
            occ = raw[idx]
            if occ < 0 or occ >= self.occupied_threshold:
                continue
            x = idx % width
            y = idx // width
            if require_unknown_neighbor and not self._has_unknown_neighbor(
                x, y, raw, width, height
            ):
                continue
            wx, wy = self._cell_to_world(x, y, origin_x, origin_y, resolution)
            dist_cur = math.hypot(wx - current_xy[0], wy - current_xy[1])
            if dist_cur < min_distance_m:
                continue
            dist_goal = math.hypot(wx - final_goal_xy[0], wy - final_goal_xy[1])
            score = dist_goal + self.frontier_distance_weight * dist_cur
            if score < best_score:
                best_score = score
                best_cell = (x, y)

        return best_cell

    def _compute_subgoal(
        self,
        current_xy: Tuple[float, float],
        final_goal_xy: Tuple[float, float],
    ) -> Tuple[Optional[Tuple[float, float]], str]:
        map_msg = self.map_msg
        if map_msg is None:
            return final_goal_xy, "direct_goal_no_map"

        width = int(map_msg.info.width)
        height = int(map_msg.info.height)
        resolution = float(map_msg.info.resolution)
        if width <= 0 or height <= 0 or resolution <= 0.0:
            return final_goal_xy, "direct_goal_bad_map"

        origin_x = float(map_msg.info.origin.position.x)
        origin_y = float(map_msg.info.origin.position.y)

        start_cell = self._world_to_cell(
            current_xy[0], current_xy[1], origin_x, origin_y, resolution
        )
        goal_cell_raw = self._world_to_cell(
            final_goal_xy[0], final_goal_xy[1], origin_x, origin_y, resolution
        )

        if not self._in_bounds(start_cell[0], start_cell[1], width, height):
            return final_goal_xy, "direct_goal_start_oob"

        blocked = self._build_blocked_grid(map_msg, start_cell)
        goal_in_bounds = self._in_bounds(goal_cell_raw[0], goal_cell_raw[1], width, height)
        if goal_in_bounds:
            goal_cell = self._nearest_free_cell(
                goal_cell_raw, blocked, width, height, resolution
            )
            if goal_cell is not None:
                path = self._a_star(start_cell, goal_cell, blocked, width, height)
                if path:
                    target_cell = self._select_path_lookahead_cell(path, resolution)
                    target_xy = self._cell_to_world(
                        target_cell[0], target_cell[1], origin_x, origin_y, resolution
                    )
                    return target_xy, "path"
        else:
            frontier_cell = self._best_frontier_cell(
                start_cell, blocked, map_msg, current_xy, final_goal_xy
            )
            if frontier_cell is not None:
                target_xy = self._cell_to_world(
                    frontier_cell[0], frontier_cell[1], origin_x, origin_y, resolution
                )
                return target_xy, "frontier_goal_oob"
            # If no frontier is visible yet, still bias motion to the best
            # currently reachable in-map free cell toward the final goal.
            reachable_cell = self._best_frontier_cell(
                start_cell,
                blocked,
                map_msg,
                current_xy,
                final_goal_xy,
                require_unknown_neighbor=False,
                min_distance_m_override=0.0,
            )
            if reachable_cell is not None:
                target_xy = self._cell_to_world(
                    reachable_cell[0], reachable_cell[1], origin_x, origin_y, resolution
                )
                return target_xy, "reachable_goal_oob"
            return final_goal_xy, "direct_goal_goal_oob_no_frontier"

        frontier_cell = self._best_frontier_cell(
            start_cell, blocked, map_msg, current_xy, final_goal_xy
        )
        if frontier_cell is not None:
            target_xy = self._cell_to_world(
                frontier_cell[0], frontier_cell[1], origin_x, origin_y, resolution
            )
            return target_xy, "frontier"

        return final_goal_xy, "direct_goal_no_subgoal"

    def _on_timer(self) -> None:
        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds / 1e9

        if not self.started:
            if elapsed < self.start_delay:
                return
            self.started = True
            self.get_logger().info("Map-driven mission started")

        if self.final_goal_xy is None:
            self.get_logger().warn(
                "Waiting for final goal on %s" % self.final_goal_topic,
                throttle_duration_sec=2.0,
            )
            return

        current_xy = self._lookup_pose_xy()
        if current_xy is None:
            self.get_logger().warn(
                "Mission waiting for TF map->base_link",
                throttle_duration_sec=2.0,
            )
            return

        dist_to_final = math.hypot(
            self.final_goal_xy[0] - current_xy[0],
            self.final_goal_xy[1] - current_xy[1],
        )
        if dist_to_final <= self.final_goal_reach_tolerance:
            if not self.final_goal_reached:
                self.final_goal_reached = True
                self.active_subgoal_xy = self.final_goal_xy
                self.get_logger().info(
                    "Final goal reached within %.2fm (distance %.2fm)"
                    % (self.final_goal_reach_tolerance, dist_to_final)
                )
            self._publish_goal(self.final_goal_xy[0], self.final_goal_xy[1])
            return
        self.final_goal_reached = False

        proposed_subgoal, mode = self._compute_subgoal(current_xy, self.final_goal_xy)
        if proposed_subgoal is None:
            self.get_logger().warn(
                "No subgoal available (%s); holding current target" % mode,
                throttle_duration_sec=2.0,
            )
            if self.active_subgoal_xy is not None:
                self._publish_goal(self.active_subgoal_xy[0], self.active_subgoal_xy[1])
            return

        if self.active_subgoal_xy is None:
            self.active_subgoal_xy = proposed_subgoal
        else:
            delta = math.hypot(
                proposed_subgoal[0] - self.active_subgoal_xy[0],
                proposed_subgoal[1] - self.active_subgoal_xy[1],
            )
            if delta >= self.retarget_min_separation_m:
                self.active_subgoal_xy = proposed_subgoal

        self._publish_goal(self.active_subgoal_xy[0], self.active_subgoal_xy[1])

        if mode != self.last_mode:
            self.last_mode = mode
            self.get_logger().info(
                "Mission mode=%s target=(%.2f, %.2f) final=(%.2f, %.2f)"
                % (
                    mode,
                    self.active_subgoal_xy[0],
                    self.active_subgoal_xy[1],
                    self.final_goal_xy[0],
                    self.final_goal_xy[1],
                )
            )

        if now.nanoseconds - self.last_status_log_ns > 2_000_000_000:
            self.last_status_log_ns = now.nanoseconds
            self.get_logger().info(
                "mission pos=(%.2f,%.2f) tgt=(%.2f,%.2f) final=(%.2f,%.2f) d_final=%.2f"
                % (
                    current_xy[0],
                    current_xy[1],
                    self.active_subgoal_xy[0],
                    self.active_subgoal_xy[1],
                    self.final_goal_xy[0],
                    self.final_goal_xy[1],
                    dist_to_final,
                )
            )


def main(args=None):
    rclpy.init(args=args)
    node = WaypointMissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
