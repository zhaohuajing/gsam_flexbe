#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyServiceCaller

from graspsam_ros2.srv import RunGraspSAM


class GraspSAMServiceState(EventState):
    """
    Calls GraspSAM ROS2 service /run_graspsam and waits for response (synchronous call).

    -- service_name      string   Service name (default: '/run_graspsam')
    -- timeout           float    Seconds to wait for service availability (default: 2.0)
    -- seen_set_default  bool     Default if userdata.seen_set is missing/None (default: False)

    ># dataset_root       string
    ># dataset_name       string
    ># checkpoint_path    string
    ># sam_encoder_type   string
    ># no_grasps          int
    ># seen_set           bool      (optional; falls back to seen_set_default)

    #> output_dir         string
    #> grasps             graspsam_ros2/Grasp[]
    #> message            string

    <= done
    <= failed
    """

    def __init__(self,
                 service_name='/run_graspsam',
                 dataset_root='',
                 dataset_name='',
                 checkpoint_path='',
                 sam_encoder_type='vit_t',
                 no_grasps=10,
                 timeout=2.0,
                 seen_set=False,
                 seen_set_default = False,
                 **_ignored_kwargs):
        super().__init__(
            outcomes=['done', 'failed'],
            input_keys=[
                'dataset_root',
                'dataset_name',
                'checkpoint_path',
                'sam_encoder_type',
                'no_grasps',
                'seen_set',
            ],
            output_keys=['output_dir', 'grasps', 'grasp_target_poses',  'message']
        )

        self._service_name = service_name
        self._timeout = float(timeout)
        self._seen_set_default = bool(seen_set_default)

        self._srv = ProxyServiceCaller({self._service_name: RunGraspSAM})

        self._sent = False
        self._success = False

    def on_enter(self, userdata):
        self._sent = False
        self._success = False
        userdata.output_dir = ''
        userdata.grasps = []
        userdata.grasp_target_poses = []
        userdata.message = ''

        # FlexBE/ProxyServiceCaller expects a FLOAT seconds wait_duration
        if not self._srv.is_available(self._service_name):
            Logger.loginfo(
                f"[GraspSAMServiceState] Waiting for service '{self._service_name}' (timeout={self._timeout}s)..."
            )
            if not self._srv.wait_for_service(self._service_name, wait_duration=self._timeout):
                userdata.message = f"Service '{self._service_name}' not available after {self._timeout}s."
                Logger.logerr(f"[GraspSAMServiceState] {userdata.message}")
                return

        # Build request
        request = RunGraspSAM.Request()
        request.dataset_root = str(userdata.dataset_root)
        request.dataset_name = str(userdata.dataset_name)
        request.checkpoint_path = str(userdata.checkpoint_path)
        request.sam_encoder_type = str(userdata.sam_encoder_type)
        request.no_grasps = int(userdata.no_grasps)

        # default seen_set=False if missing/None
        seen = getattr(userdata, 'seen_set', None)
        request.seen_set = self._seen_set_default if seen is None else bool(seen)

        Logger.loginfo(
            "[GraspSAMServiceState] Calling {} with dataset_root={}, dataset_name={}, checkpoint={}, encoder={}, no_grasps={}, seen_set={}".format(
                self._service_name,
                request.dataset_root,
                request.dataset_name,
                request.checkpoint_path,
                request.sam_encoder_type,
                request.no_grasps,
                request.seen_set
            )
        )

        # Synchronous call (CGN-style)
        try:
            resp = self._srv.call(self._service_name, request)
            self._sent = True
        except Exception as e:
            userdata.message = f"Service call exception: {e}"
            Logger.logerr(f"[GraspSAMServiceState] {userdata.message}")
            return

        if resp is None:
            userdata.message = "Service call returned None."
            Logger.logerr(f"[GraspSAMServiceState] {userdata.message}")
            return

        userdata.message = resp.message
        userdata.output_dir = resp.output_dir
        userdata.grasps = list(resp.grasps) if resp.grasps is not None else []

        # Extract base-frame target poses (Pose list) from grasps
        poses = []
        for g in userdata.grasps:
            if hasattr(g, 'pose_base'):
                pb = g.pose_base
                # defensive: skip uninitialized/default poses if needed
                poses.append(pb)
        userdata.grasp_target_poses = poses

        self._success = bool(resp.success)

        if self._success:
            Logger.loginfo(
                f"[GraspSAMServiceState] Success. Parsed grasps: {len(userdata.grasps)}  output_dir: {userdata.output_dir}"
            )
        else:
            Logger.logerr(f"[GraspSAMServiceState] Failure: {userdata.message}")

    def execute(self, userdata):
        # If we never even sent successfully, fail
        if not self._sent:
            return 'failed'
        return 'done' if self._success else 'failed'
