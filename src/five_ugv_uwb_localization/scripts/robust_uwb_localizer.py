#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function
import math
import threading
from collections import deque
try:
    from time import monotonic
except ImportError:
    # ROS Melodic/Python 2 on Linux: use the native monotonic clock.
    import ctypes

    class _Timespec(ctypes.Structure):
        _fields_ = [("tv_sec", ctypes.c_long), ("tv_nsec", ctypes.c_long)]

    _clock_gettime = ctypes.CDLL("librt.so.1", use_errno=True).clock_gettime
    _clock_gettime.argtypes = [ctypes.c_int, ctypes.POINTER(_Timespec)]
    _clock_gettime.restype = ctypes.c_int

    def monotonic():
        value = _Timespec()
        if _clock_gettime(1, ctypes.byref(value)) != 0:  # CLOCK_MONOTONIC
            raise OSError(ctypes.get_errno(), "clock_gettime(CLOCK_MONOTONIC) failed")
        return value.tv_sec + value.tv_nsec*1e-9
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, PointStamped
from nav_msgs.msg import Path
from std_msgs.msg import Bool, Float64, Int32, Int32MultiArray
from nlink_parser.msg import LinktrackNodeframe2

class RobustUWBLocalizer(object):
    def __init__(self):
        self.input_topic = rospy.get_param("~input_topic")
        self.world_frame = rospy.get_param("~world_frame", "uwb_map")
        self.tag_z = float(rospy.get_param("~tag_height", 0.25))

        self.anchors = {}
        self.order = []
        for a in rospy.get_param("~anchors", []):
            aid = int(a["id"])
            self.anchors[aid] = np.array([float(a["x"]), float(a["y"]), float(a["z"])], dtype=float)
            self.order.append(aid)

        self.min_anchors = int(rospy.get_param("~min_anchors", 4))
        self.range_timeout = float(rospy.get_param("~range_timeout", 0.12))
        self.min_range = float(rospy.get_param("~min_valid_range", 1.05))
        self.max_range = float(rospy.get_param("~max_valid_range", 20.0))
        self.sigma = float(rospy.get_param("~range_sigma", 0.10))
        self.huber_delta = float(rospy.get_param("~huber_delta", 0.25))
        self.max_iter = int(rospy.get_param("~max_iterations", 10))
        self.tol = float(rospy.get_param("~convergence_tolerance", 1e-4))
        self.max_step = float(rospy.get_param("~max_gn_step", 1.0))

        self.jump_threshold = float(rospy.get_param("~jump_threshold", 0.40))
        self.jump_max_rate = float(rospy.get_param("~jump_max_rate", 1.0))
        self.jump_reset_time = float(rospy.get_param("~jump_reset_time", 0.50))

        self.loo_trigger = float(rospy.get_param("~loo_trigger_rms", 0.30))
        self.loo_abs = float(rospy.get_param("~loo_min_absolute_improvement", 0.08))
        self.loo_ratio = float(rospy.get_param("~loo_max_rms_ratio", 0.75))

        self.alpha = float(rospy.get_param("~position_lpf_alpha", 0.35))
        self.lpf_reference_rate = float(rospy.get_param("~position_lpf_reference_rate", 50.0))
        self.xmin = float(rospy.get_param("~workspace_x_min", 0.0))
        self.xmax = float(rospy.get_param("~workspace_x_max", 6.4))
        self.ymin = float(rospy.get_param("~workspace_y_min", 0.0))
        self.ymax = float(rospy.get_param("~workspace_y_max", 4.4))
        self.margin = float(rospy.get_param("~workspace_margin", 0.8))

        self.max_position_jump = float(rospy.get_param("~max_position_jump", 0.35))
        self.position_jump_reset_time = float(rospy.get_param("~position_jump_reset_time", 0.50))
        self.valid_max_rms = float(rospy.get_param("~valid_max_residual_rms", 0.35))
        self.watchdog_timeout = float(rospy.get_param("~input_watchdog_timeout", 0.20))
        self.solve_rate = float(rospy.get_param("~solve_rate", 20.0))
        self.max_measurement_age = float(rospy.get_param("~max_measurement_age", 0.15))
        self.path_min_distance = float(rospy.get_param("~path_min_distance", 0.03))
        self.path_publish_rate = float(rospy.get_param("~path_publish_rate", 2.0))
        self.path_max_points = int(rospy.get_param("~path_max_points", 1000))

        for name in ("solve_rate", "max_measurement_age", "lpf_reference_rate",
                     "watchdog_timeout", "range_timeout"):
            value = getattr(self, name)
            if not self.finite(value) or value <= 0.0:
                raise ValueError("%s must be finite and positive" % name)
        if not 0.0 <= self.alpha <= 1.0:
            raise ValueError("position_lpf_alpha must be between 0 and 1")
        if not self.finite(self.path_publish_rate) or self.path_publish_rate < 0.0:
            raise ValueError("path_publish_rate must be finite and nonnegative")
        if self.path_max_points < 1:
            raise ValueError("path_max_points must be positive")

        self.input_lock = threading.Lock()
        self.output_lock = threading.Lock()
        self.latest_frame = None
        self.dropped_frames = 0
        self.meas = {}
        self.last_good_range = {}
        self.raw_solution = None
        self.filtered = None
        self.last_valid_time = None
        self.last_input_time = None
        self.last_path_xy = None
        self.path_poses = deque(maxlen=self.path_max_points)
        self.last_path_publish_time = None
        self.path_dirty = False

        self.pose_pub = rospy.Publisher("uwb/pose", PoseStamped, queue_size=1)
        self.point_pub = rospy.Publisher("uwb/point", PointStamped, queue_size=1)
        self.path_pub = rospy.Publisher("uwb/path", Path, queue_size=1, latch=True)
        self.valid_pub = rospy.Publisher("uwb/valid", Bool, queue_size=1, latch=True)
        self.rms_pub = rospy.Publisher("uwb/residual_rms", Float64, queue_size=1)
        self.full_rms_pub = rospy.Publisher("uwb/full_residual_rms", Float64, queue_size=1)
        self.count_pub = rospy.Publisher("uwb/used_anchor_count", Int32, queue_size=1)
        self.jump_pub = rospy.Publisher("uwb/jump_rejected_ids", Int32MultiArray, queue_size=1)
        self.loo_pub = rospy.Publisher("uwb/loo_excluded_id", Int32, queue_size=1)
        self.age_pub = rospy.Publisher("uwb/measurement_age", Float64, queue_size=1)
        self.processing_pub = rospy.Publisher("uwb/processing_time", Float64, queue_size=1)
        self.dropped_pub = rospy.Publisher("uwb/dropped_frame_count", Int32, queue_size=1)

        self.publish_valid(False)
        self.input_sub = rospy.Subscriber(self.input_topic, LinktrackNodeframe2, self.cb,
                                          queue_size=1, buff_size=2**20)
        self.solve_timer = rospy.Timer(rospy.Duration(1.0/self.solve_rate), self.process_cb)
        self.watchdog_timer = rospy.Timer(rospy.Duration(0.05), self.watchdog_cb)
        rospy.loginfo("Algorithm-B UWB localizer started in %s", rospy.get_namespace())
        rospy.loginfo("Input: %s", rospy.resolve_name(self.input_topic))
        rospy.loginfo("Latest-frame solving at %.1f Hz; maximum measurement age %.3f s",
                      self.solve_rate, self.max_measurement_age)

    def finite(self, v):
        return not (math.isnan(v) or math.isinf(v))

    def physical_valid(self, aid, r):
        if not self.finite(r) or r < self.min_range or r > self.max_range:
            return False
        dz = abs(self.tag_z - self.anchors[aid][2])
        return not (r + 0.05 < dz)

    def cb(self, msg):
        # Keep reception independent of WLS/LOO and message serialization.
        # ponytail: receipt time only; upstream latency needs a driver timestamp.
        with self.input_lock:
            stamp, received_at = rospy.Time.now(), monotonic()
            if self.latest_frame is not None:
                self.dropped_frames += 1
            self.latest_frame = (msg, stamp, received_at)
            self.last_input_time = received_at

    def process_cb(self, _event):
        # One Timer owns all range/solver/filter/path state. Never hold this
        # lock during computation: incoming frames must be able to replace it.
        with self.input_lock:
            frame = self.latest_frame
            self.latest_frame = None
        if frame is None:
            return

        msg, stamp, received_at = frame
        started = monotonic()
        try:
            if started - received_at > self.max_measurement_age:
                self.publish_valid(False)
                rospy.logwarn_throttle(1.0, "Skipping stale UWB input (age %.3f s)",
                                       started - received_at)
                return
            self.process_ranges(msg, received_at)
            self.solve(stamp, received_at)
        finally:
            finished = monotonic()
            with self.input_lock:
                dropped = self.dropped_frames
            self.age_pub.publish(Float64(data=finished - received_at))
            self.processing_pub.publish(Float64(data=finished - started))
            self.dropped_pub.publish(Int32(data=dropped))

    def process_ranges(self, msg, now):
        jump_ids = []

        for node in msg.nodes:
            aid = int(node.id)
            if aid not in self.anchors:
                continue
            r = float(node.dis)
            if not self.physical_valid(aid, r):
                continue

            accept = True
            if aid in self.last_good_range:
                prev_r, prev_t = self.last_good_range[aid]
                dt = now - prev_t
                if dt <= self.jump_reset_time:
                    allowed = self.jump_threshold + self.jump_max_rate * max(dt, 0.0)
                    if abs(r - prev_r) > allowed:
                        accept = False

            if accept:
                self.meas[aid] = {"id": aid, "range": r, "stamp": now, "anchor": self.anchors[aid]}
                self.last_good_range[aid] = (r, now)
            else:
                jump_ids.append(aid)

        m = Int32MultiArray()
        m.data = jump_ids
        self.jump_pub.publish(m)

    def fresh(self, now):
        data = []
        for aid in self.order:
            if aid in self.meas and 0.0 <= now - self.meas[aid]["stamp"] <= self.range_timeout:
                data.append(self.meas[aid])
        return data

    def linear_ls(self, data):
        ref = data[0]
        r0 = ref["range"]
        x0,y0,z0 = ref["anchor"]
        A=[]; b=[]
        for item in data[1:]:
            ri=item["range"]; xi,yi,zi=item["anchor"]
            A.append([2*(xi-x0),2*(yi-y0)])
            b.append(r0*r0-ri*ri+xi*xi-x0*x0+yi*yi-y0*y0+(self.tag_z-zi)**2-(self.tag_z-z0)**2)
        try:
            x = np.linalg.lstsq(np.asarray(A), np.asarray(b), rcond=-1)[0]
            return x if np.all(np.isfinite(x)) else None
        except Exception:
            return None

    def huber(self, e):
        a=abs(e)
        if self.huber_delta <= 0 or a <= self.huber_delta:
            return 1.0
        return self.huber_delta/max(a,1e-9)

    def wls(self, initial, data):
        if len(data) < self.min_anchors:
            return None, None
        x=np.asarray(initial,dtype=float).copy()
        for _ in range(self.max_iter):
            H=[]; r=[]; w=[]
            for item in data:
                ax,ay,az=item["anchor"]
                dx=x[0]-ax; dy=x[1]-ay; dz=self.tag_z-az
                pred=max(math.sqrt(dx*dx+dy*dy+dz*dz),1e-6)
                e=item["range"]-pred
                H.append([dx/pred,dy/pred]); r.append(e)
                w.append(self.huber(e)/max(self.sigma*self.sigma,1e-9))
            H=np.asarray(H); r=np.asarray(r); W=np.diag(np.asarray(w))
            normal=H.T.dot(W).dot(H); rhs=H.T.dot(W).dot(r)
            try:
                delta=np.linalg.solve(normal,rhs)
            except Exception:
                try:
                    delta=np.linalg.lstsq(normal,rhs,rcond=-1)[0]
                except Exception:
                    return None,None
            step=float(np.linalg.norm(delta))
            if step > self.max_step:
                delta *= self.max_step/max(step,1e-9)
            x += delta
            if float(np.linalg.norm(delta)) < self.tol:
                break
        return x,self.rms(x,data)

    def rms(self, x, data):
        errs=[]
        for item in data:
            ax,ay,az=item["anchor"]
            pred=math.sqrt((x[0]-ax)**2+(x[1]-ay)**2+(self.tag_z-az)**2)
            errs.append(item["range"]-pred)
        return math.sqrt(sum(e*e for e in errs)/float(len(errs)))

    def inside(self, x):
        return self.xmin-self.margin <= x[0] <= self.xmax+self.margin and self.ymin-self.margin <= x[1] <= self.ymax+self.margin

    def loo(self, base_x, base_rms, data):
        if base_rms <= self.loo_trigger or len(data) <= self.min_anchors:
            return base_x,base_rms,data,-1
        best=None
        for k in range(len(data)):
            subset=data[:k]+data[k+1:]
            cx,cr=self.wls(base_x,subset)
            if cx is None or not self.inside(cx):
                continue
            if best is None or cr < best[1]:
                best=(cx,cr,subset,data[k]["id"])
        if best is None:
            return base_x,base_rms,data,-1
        if (base_rms-best[1] >= self.loo_abs and best[1]/max(base_rms,1e-9) <= self.loo_ratio):
            return best
        return base_x,base_rms,data,-1

    def solve(self, stamp, received_at):
        data=self.fresh(monotonic())
        if len(data) < self.min_anchors:
            self.publish_valid(False); return

        init=self.raw_solution.copy() if self.raw_solution is not None else self.linear_ls(data)
        if init is None:
            self.publish_valid(False); return

        bx,br=self.wls(init,data)
        if bx is None or not self.inside(bx):
            self.publish_valid(False); return

        fx,fr,used,loo_id=self.loo(bx,br,data)
        full_rms=self.rms(fx,data)

        lm=Int32(); lm.data=int(loo_id); self.loo_pub.publish(lm)
        cm=Int32(); cm.data=len(used); self.count_pub.publish(cm)
        rm=Float64(); rm.data=float(fr); self.rms_pub.publish(rm)
        fm=Float64(); fm.data=float(full_rms); self.full_rms_pub.publish(fm)

        dt = None if self.last_valid_time is None else received_at - self.last_valid_time
        if self.raw_solution is not None and dt is not None:
            if dt <= self.position_jump_reset_time:
                if float(np.linalg.norm(fx-self.raw_solution)) > self.max_position_jump:
                    self.publish_valid(False); return

        if fr > self.valid_max_rms:
            self.publish_valid(False); return

        # Do not commit an expired result, even if fresh inputs arrived while
        # it was being computed. Serialize this check/publication with watchdog.
        with self.output_lock:
            finished = monotonic()
            if (finished - received_at > self.max_measurement_age or
                    any(finished - item["stamp"] > self.range_timeout for item in used)):
                self.publish_valid(False)
                rospy.logwarn_throttle(1.0, "Discarding expired UWB solution (age %.3f s)",
                                       finished - received_at)
                return
            self.raw_solution = fx.copy()
            alpha = self.filter_alpha(dt) if dt is not None else 1.0
            self.filtered = fx.copy() if self.filtered is None else alpha*fx+(1-alpha)*self.filtered
            self.last_valid_time = received_at

            pose=PoseStamped()
            pose.header.stamp=stamp; pose.header.frame_id=self.world_frame
            pose.pose.position.x=float(self.filtered[0]); pose.pose.position.y=float(self.filtered[1]); pose.pose.position.z=self.tag_z
            pose.pose.orientation.w=1.0
            self.pose_pub.publish(pose)

            pt=PointStamped(); pt.header=pose.header; pt.point=pose.pose.position; self.point_pub.publish(pt)
            self.publish_valid(True)

        self.update_path(pose)

        rospy.loginfo_throttle(0.5,"UWB pose x=%.3f y=%.3f valid=1 anchors=%d rms=%.3f loo=%d",
                               self.filtered[0],self.filtered[1],len(used),fr,loo_id)

    def filter_alpha(self, dt):
        # Preserve the nominal filter time constant when frames are skipped.
        return 1.0 - (1.0 - self.alpha)**(max(dt, 0.0)*self.lpf_reference_rate)

    def update_path(self, pose):
        if self.path_publish_rate == 0.0:
            return
        xy = (pose.pose.position.x, pose.pose.position.y)
        if self.last_path_xy is None or math.hypot(xy[0]-self.last_path_xy[0], xy[1]-self.last_path_xy[1]) >= self.path_min_distance:
            self.last_path_xy = xy
            self.path_poses.append(pose)
            self.path_dirty = True
        now = monotonic()
        if self.path_dirty and (self.last_path_publish_time is None or
                               now - self.last_path_publish_time >= 1.0/self.path_publish_rate):
            path = Path()
            path.header = self.path_poses[-1].header
            path.poses = list(self.path_poses)
            self.path_pub.publish(path)
            self.last_path_publish_time = monotonic()
            self.path_dirty = False

    def publish_valid(self, value):
        m=Bool(); m.data=bool(value); self.valid_pub.publish(m)

    def watchdog_cb(self,_):
        with self.output_lock:
            with self.input_lock:
                last_input = self.last_input_time
            now = monotonic()
            if (last_input is None or now - last_input > self.watchdog_timeout or
                    self.last_valid_time is None or
                    now - self.last_valid_time > self.max_measurement_age):
                self.publish_valid(False)

if __name__ == "__main__":
    rospy.init_node("uwb_localizer")
    RobustUWBLocalizer()
    rospy.spin()
