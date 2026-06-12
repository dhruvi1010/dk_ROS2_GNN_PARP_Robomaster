#!/usr/bin/env python3
"""
route_cost_puc_pynode  —  Option 2, Phase 3 (Day 1).

Subscribes to the Nav2 global plan and scores it with route cost J(pi) and PUC.
Day 1: ETA (time) + Energy terms real; observability/comms/safety = 0.0 stubs.

All inputs are in the `map` frame (plan, polygons, costmap) — no TF needed.
Topic names are relative so PushRosNamespace('rm03') resolves them under /rm03.
"""
import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import Path, OccupancyGrid
from sensor_msgs.msg import BatteryState
from gnn_interfaces.msg import TrackedPolygon
from perception_aware_nav2_msgs.msg import LinkStats, RouteCost, RoutePUC, RoutePUCComponents

from route_cost_puc_pynode.path_sampler import resample_path
from route_cost_puc_pynode.polygon_tracker import PolygonTracker


def sigmoid(x):
    if x > 50:  return 1.0
    if x < -50: return 0.0
    return 1.0 / (1.0 + math.exp(-x))


def norm_cdf(x):
    # P(Z <= x) without scipy
    return 0.5 * math.erfc(-x / math.sqrt(2.0))


class RouteCostPucNode(Node):
    def __init__(self):
        super().__init__('route_cost_puc')

        # ---------------- parameters ----------------
        self.declare_parameter('robot_id', 'rm03')
        self.declare_parameter('trial_id', '')
        self.declare_parameter('plan_topic', '/rm03/plan')         # absolute, NavfnPlanner output
        self.declare_parameter('polygons_topic', '/tracked_polygons')
        self.declare_parameter('link_stats_topic', 'comms/link_stats')
        self.declare_parameter('battery_topic', 'battery_state')
        self.declare_parameter('costmap_topic', 'global_costmap/costmap')

        # time / reliability
        self.declare_parameter('v_ref_mps', 0.6)                   # = FollowPath.max_vel_x
        self.declare_parameter('k_slowdown', 0.01)
        self.declare_parameter('resample_ds_m', 0.10)              # 2x costmap res
        self.declare_parameter('t_sla_s', 30.0)
        self.declare_parameter('sigma_T_per_meter', 0.05)
        self.declare_parameter('sigma_T_per_ms_jitter', 0.01)

        # observability
        self.declare_parameter('tube_radius_m', 1.0)
        self.declare_parameter('obs_a_uncertainty', 1.0)
        self.declare_parameter('obs_b_fragility', 0.7)
        self.declare_parameter('obs_decay_tau_m', 0.8)

        # comms
        self.declare_parameter('rsrp_min_dbm', -85.0)
        self.declare_parameter('rsrp_scale_db', 10.0)
        self.declare_parameter('jitter_max_ms', 30.0)
        self.declare_parameter('jitter_scale_ms', 15.0)
        self.declare_parameter('w_rsrp_obs', 0.6)
        self.declare_parameter('w_jit_obs', 0.4)

        # safety Tier-A
        self.declare_parameter('safety_horizon_s', 3.0)
        self.declare_parameter('dynamic_labels', [2, 4])
        self.declare_parameter('safety_g_min_s', 1.0)
        self.declare_parameter('safety_scale_s', 0.4)
        self.declare_parameter('match_radius_m', 0.4)
        self.declare_parameter('decay_time_s', 20.0)
        self.declare_parameter('safety_history_window_s', 2.0)

        # energy
        self.declare_parameter('use_battery_topic', True)
        self.declare_parameter('energy_alpha_per_m', 1.0)
        self.declare_parameter('energy_beta_per_turn', 0.5)
        self.declare_parameter('battery_capacity_wh', 0.0)         # 0 => energy observational
        self.declare_parameter('energy_reserve_wh', 2.0)
        self.declare_parameter('energy_scale_wh', 5.0)
        self.declare_parameter('turn_threshold_rad', 0.20)         # yaw step counted as a "turn"

        # J(pi) weights
        self.declare_parameter('lambda_obs', 1.0)
        self.declare_parameter('lambda_comms', 1.0)
        self.declare_parameter('lambda_safety', 2.0)
        self.declare_parameter('lambda_energy', 0.0)

        # PUC fusion weights
        self.declare_parameter('w_t', 1.0)
        self.declare_parameter('w_o', 1.0)
        self.declare_parameter('w_c', 1.0)
        self.declare_parameter('w_s', 2.0)
        self.declare_parameter('w_e', 0.5)             # 0 until battery_capacity_wh set

        gp = self.get_parameter
        self.robot_id = gp('robot_id').value
        self.trial_id = gp('trial_id').value
        self.v_ref = float(gp('v_ref_mps').value)
        self.k_slowdown = float(gp('k_slowdown').value)
        self.ds = float(gp('resample_ds_m').value)
        self.t_sla = float(gp('t_sla_s').value)
        self.sigT_L = float(gp('sigma_T_per_meter').value)
        self.sigT_j = float(gp('sigma_T_per_ms_jitter').value)
        self.tube_radius_m = float(gp('tube_radius_m').value)
        self.obs_a = float(gp('obs_a_uncertainty').value)
        self.obs_b = float(gp('obs_b_fragility').value)
        self.obs_decay_tau_m = float(gp('obs_decay_tau_m').value)
        self.rsrp_min_dbm = float(gp('rsrp_min_dbm').value)
        self.rsrp_scale_db = float(gp('rsrp_scale_db').value)
        self.jitter_max_ms = float(gp('jitter_max_ms').value)
        self.jitter_scale_ms = float(gp('jitter_scale_ms').value)
        self.w_rsrp_obs = float(gp('w_rsrp_obs').value)
        self.w_jit_obs = float(gp('w_jit_obs').value)
        self.safety_horizon_s = float(gp('safety_horizon_s').value)
        self.dynamic_labels = [int(x) for x in gp('dynamic_labels').value]
        self.safety_g_min_s = float(gp('safety_g_min_s').value)
        self.safety_scale_s = float(gp('safety_scale_s').value)
        self.match_radius_m = float(gp('match_radius_m').value)
        self.decay_time_s = float(gp('decay_time_s').value)
        self.safety_history_window_s = float(gp('safety_history_window_s').value)
        self.use_batt = bool(gp('use_battery_topic').value)
        self.e_alpha = float(gp('energy_alpha_per_m').value)
        self.e_beta = float(gp('energy_beta_per_turn').value)
        self.batt_cap = float(gp('battery_capacity_wh').value)
        self.e_reserve = float(gp('energy_reserve_wh').value)
        self.e_scale = float(gp('energy_scale_wh').value)
        self.turn_thr = float(gp('turn_threshold_rad').value)
        self.lam_o = float(gp('lambda_obs').value)
        self.lam_c = float(gp('lambda_comms').value)
        self.lam_s = float(gp('lambda_safety').value)
        self.lam_e = float(gp('lambda_energy').value)
        self.w = {k: float(gp('w_' + k).value) for k in ('t', 'o', 'c', 's', 'e')}
        
        # ---------------- state ----------------
        self.costmap = None
        self.jitter_ms = None
        self.rsrp_dbm = None
        self.soc = 1.0              # latest battery percentage [0,1]
        self.poly_tracker = PolygonTracker(
            match_radius_m=self.match_radius_m,
            decay_time_s=self.decay_time_s,
            history_window_s=self.safety_history_window_s,
        )
              

        # ---------------- QoS ----------------
        # Costmap is latched (transient_local) in Nav2; match it.        
        qos_map = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             history=HistoryPolicy.KEEP_LAST)


        # ---------------- subs ----------------
        self.create_subscription(Path, gp('plan_topic').value, self.on_plan, 10)
        self.create_subscription(OccupancyGrid, gp('costmap_topic').value,
                                 self.on_costmap, qos_map)
        self.create_subscription(LinkStats, gp('link_stats_topic').value,
                                 self.on_link, 10)
        self.create_subscription(TrackedPolygon, gp('polygons_topic').value,
                                 self.on_polygon, 10)
        if self.use_batt:
            self.create_subscription(BatteryState, gp('battery_topic').value,
                                     self.on_battery, 10)
            
        # ---------------- pubs ----------------
        self.pub_cost = self.create_publisher(RouteCost, 'route_cost', 10)
        self.pub_puc = self.create_publisher(RoutePUC, 'route_puc', 10)
        self.pub_comp = self.create_publisher(RoutePUCComponents, 'route_puc_components', 10)

        self.get_logger().info(
    f"\033[32m[route_cost_puc] up... v_ref={self.v_ref} ds={self.ds} "
    f"tube={self.tube_radius_m} H_safe={self.safety_horizon_s} "
    f"dyn_labels={self.dynamic_labels} batt_cap={self.batt_cap} "
    f"λ_e={self.lam_e}\033[0m")


    # ---------- input callbacks ----------
    def on_costmap(self, msg):  self.costmap = msg
    def on_link(self, msg):     
        self.jitter_ms = float(msg.jitter_ms) 
        self.rsrp_dbm = float(msg.rsrp_dbm)
    def on_battery(self, msg):
        # percentage is the trustable SoC field on this driver (see Lf3_Lf4 §2.3)
        if 0.0 <= msg.percentage <= 1.0:
            self.soc = float(msg.percentage)
    def on_polygon(self, msg):
        # Guard against non-map frames (Lf3_Lf4 §2.5: confirmed `map` in practice).
        if msg.header.frame_id and msg.header.frame_id != 'map':
            return
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        self.poly_tracker.update(msg, now_sec)
       

    # ---------- costmap sampling ----------
    def cost_at(self, x, y):
        cm = self.costmap
        if cm is None:
            return 0.0
        res = cm.info.resolution
        ox = cm.info.origin.position.x
        oy = cm.info.origin.position.y
        ci = int((x - ox) / res)
        cj = int((y - oy) / res)
        if ci < 0 or cj < 0 or ci >= cm.info.width or cj >= cm.info.height:
            return 0.0
        v = cm.data[cj * cm.info.width + ci]
        return 0.0 if v < 0 else float(v)   # -1 = unknown -> treat as free



    # ---------- term computations ----------
    def compute_obs_term(self, rx, ry, seg_ds):
        tracks = self.poly_tracker.active()
        if not tracks or len(rx) < 2:
            return 0.0, 1.0
        tube2 = self.tube_radius_m ** 2
        tau = max(1e-3, self.obs_decay_tau_m)
        mids_x = 0.5 * (rx[:-1] + rx[1:])
        mids_y = 0.5 * (ry[:-1] + ry[1:])
        s = 0.0
        for ds, mx, my in zip(seg_ds, mids_x, mids_y):
            rho_o = 0.0
            for tr in tracks:
                dx, dy = tr['cx'] - mx, tr['cy'] - my
                d2 = dx*dx + dy*dy
                if d2 > tube2: continue
                d = d2 ** 0.5
                risk_k = self.poly_tracker.risk(tr, self.obs_a, self.obs_b)
                rho_o += risk_k * math.exp(-d / tau)
            s += rho_o * ds
        return float(s), float(math.exp(-s))

    def compute_comms_term(self, path_length_m):
        if self.rsrp_dbm is None or self.jitter_ms is None:
            return 0.0, 1.0
        span_r = max(1e-3, self.rsrp_scale_db)
        span_j = max(1e-3, self.jitter_scale_ms)
        r_rsrp = max(0.0, min(1.0, (self.rsrp_min_dbm - self.rsrp_dbm) / span_r))
        r_jit = max(0.0, min(1.0, (self.jitter_ms - self.jitter_max_ms) / span_j))
        rho_c = self.w_rsrp_obs * r_rsrp + self.w_jit_obs * r_jit
        sum_rho_c = rho_c * path_length_m
        p_rsrp = sigmoid((self.rsrp_dbm - self.rsrp_min_dbm) / span_r)
        p_jit = sigmoid((self.jitter_max_ms - self.jitter_ms) / span_j)
        p_c = max(0.0, min(1.0, p_rsrp * p_jit))
        return float(sum_rho_c), float(p_c)

    def compute_safety_term(self, rx, ry, ryaw, seg_ds, now_sec):
        if len(rx) < 2: return 0.0, 1.0
        cum_t = [0.0]
        mids_x = 0.5 * (rx[:-1] + rx[1:])
        mids_y = 0.5 * (ry[:-1] + ry[1:])
        for ds, mx, my in zip(seg_ds, mids_x, mids_y):
            c = self.cost_at(float(mx), float(my))
            v = max(1e-3, self.v_ref * math.exp(-self.k_slowdown * c))
            cum_t.append(cum_t[-1] + ds / v)
        dyn = set(self.dynamic_labels)
        dyn_tracks = [tr for tr in self.poly_tracker.active() if tr['label'] in dyn]
        if not dyn_tracks: return 0.0, 1.0

        H = self.safety_horizon_s
        gap_min = float('inf')
        for tr in dyn_tracks:
            vx, vy = self.poly_tracker.velocity(tr, now_sec)
            for i, t in enumerate(cum_t):
                if t > H: break
                ox = tr['cx'] + vx * t
                oy = tr['cy'] + vy * t
                dx = ox - rx[i]
                dy = oy - ry[i]
                d = (dx*dx + dy*dy) ** 0.5
                yaw = float(ryaw[i])
                vrx = vx - self.v_ref * math.cos(yaw)
                vry = vy - self.v_ref * math.sin(yaw)
                if d < 1e-6:
                    gap_min = 0.0
                    continue
                closing = -(vrx*dx + vry*dy) / d
                if closing <= 1e-3: continue
                ttc = d / closing
                if ttc < gap_min: gap_min = ttc
        if gap_min == float('inf'): return 0.0, 1.0
        p_s = sigmoid((gap_min - self.safety_g_min_s) / max(1e-3, self.safety_scale_s))
        eps = 1e-6
        rho_s = -math.log(p_s + eps)
        return float(rho_s), float(p_s)
    
    # ---------- main compute ----------
    def on_plan(self, msg: Path):
        if len(msg.poses) < 2:      # can't score trivial paths (also guards against empty plans on startup)
            return
        
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        self.poly_tracker.prune(now_sec)

        # Plan is in map (inner pose headers are empty — do NOT trust them).
        xs = np.array([p.pose.position.x for p in msg.poses])
        ys = np.array([p.pose.position.y for p in msg.poses])
        s, rx, ry, ryaw = resample_path(xs, ys, self.ds)
        if len(s) < 2:
            return
        path_len = float(s[-1])
        seg_ds = np.diff(s)


        # ----- ETA term (Step E) -----
        # v(x) = v_ref * exp(-k * cost(x)); mu_T = sum( ds / v )
        mids_x = 0.5 * (rx[:-1] + rx[1:])
        mids_y = 0.5 * (ry[:-1] + ry[1:])
        mu_T = 0.0
        for ds_i, mx, my in zip(seg_ds, mids_x, mids_y):
            c = self.cost_at(float(mx), float(my))
            v = max(1e-3, self.v_ref * math.exp(-self.k_slowdown * c))
            mu_T += ds_i / v
        time_term = mu_T
        jitter_for_sigma = self.jitter_ms if self.jitter_ms is not None else 0.0
        
        sigma_T = max(1e-3, self.sigT_L * path_len + self.sigT_j * jitter_for_sigma)
        p_t = norm_cdf((self.t_sla - mu_T) / sigma_T)   # P(T <= SLA)

        # ----- Energy term (Step F) -----
        dyaw = np.abs(np.diff(ryaw))
        dyaw = np.minimum(dyaw, 2 * math.pi - dyaw)     # wrap
        n_turns = int(np.sum(dyaw > self.turn_thr))
        e_path = self.e_alpha * path_len + self.e_beta * n_turns   # Wh
        energy_term = e_path

        if self.batt_cap > 0.0:
            e_remaining_now = self.soc * self.batt_cap
            e_remaining_goal = e_remaining_now - e_path
            p_e = sigmoid((e_remaining_goal - self.e_reserve) / self.e_scale)
        else:
            p_e = 1.0   # observational mode: energy doesn't gate

        # -----new  obs/comms/safety stubs  -----
        obs_term, p_o = self.compute_obs_term(rx, ry, seg_ds)
        comms_term, p_c = self.compute_comms_term(path_len)
        safety_term, p_s = self.compute_safety_term(rx, ry, ryaw, seg_ds, now_sec)

        j_total = (time_term
                   + self.lam_o * obs_term
                   + self.lam_c * comms_term
                   + self.lam_s * safety_term
                   + self.lam_e * energy_term)

        w = self.w
        wsum = sum(w.values())
        comps = {'t': p_t, 'o': p_o, 'c': p_c, 's': p_s, 'e': p_e}
        eps = 1e-6
        log_puc = sum(w[k] * math.log(max(eps, comps[k])) for k in comps) / max(eps, wsum)
        puc = math.exp(log_puc)


        # publish
        now = self.get_clock().now().to_msg()
        rc = RouteCost()
        rc.header.stamp = now
        rc.header.frame_id = 'map'
        rc.robot_id = self.robot_id
        rc.trial_id = self.trial_id
        rc.j_total = j_total
        rc.time_term = time_term
        rc.obs_term = obs_term
        rc.comms_term = comms_term
        rc.safety_term = safety_term
        rc.energy_term = energy_term
        rc.lambda_obs = self.lam_o
        rc.lambda_comms = self.lam_c
        rc.lambda_safety = self.lam_s
        rc.lambda_energy = self.lam_e
        rc.path_length_m = path_len
        rc.nominal_speed_mps = self.v_ref
        rc.soc_fraction = self.soc
        self.pub_cost.publish(rc)

        comp = RoutePUCComponents()
        comp.header.stamp = now
        comp.robot_id = self.robot_id
        comp.trial_id = self.trial_id
        comp.p_t, comp.p_o, comp.p_c, comp.p_s, comp.p_e = p_t, p_o, p_c, p_s, p_e
        self.pub_comp.publish(comp)

        pm = RoutePUC()
        pm.header.stamp = now
        pm.robot_id = self.robot_id
        pm.trial_id = self.trial_id
        pm.puc = puc
        pm.w_t, pm.w_o, pm.w_c, pm.w_s, pm.w_e = w['t'], w['o'], w['c'], w['s'], w['e']
        self.pub_puc.publish(pm)

        self.get_logger().info(
    f"\033[32mJ={j_total:.2f} (T={time_term:.2f} O={obs_term:.2f} C={comms_term:.2f} "
    f"S={safety_term:.2f} E={energy_term:.2f}) L={path_len:.2f}m  "
    f"PUC={puc:.3f} (pt={p_t:.2f} po={p_o:.2f} pc={p_c:.2f} ps={p_s:.2f} pe={p_e:.2f})\033[0m",
    throttle_duration_sec=2.0)


        # ----- PUC (geometric mean) -----
        w = self.w
        wsum = sum(w.values())
        comps = {'t': p_t, 'o': p_o, 'c': p_c, 's': p_s, 'e': p_e}
        eps = 1e-6
        log_puc = sum(w[k] * math.log(max(eps, comps[k])) for k in comps) / max(eps, wsum)
        puc = math.exp(log_puc)


def main(args=None):
    rclpy.init(args=args)
    node = RouteCostPucNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()