from __future__ import annotations

import json
import threading
import time
import unittest
from urllib.error import HTTPError
from urllib.request import Request, urlopen

from openflex_isaac_sensors.runtime_control import (
    RuntimeSensorControlServer,
    RuntimeSensorManager,
    RuntimeSensorResource,
)
from openflex_isaac_sensors.runtime_sensors import (
    RobotSensorRuntime,
    _CameraPublisherBootstrap,
)


class RuntimeSensorControlTest(unittest.TestCase):
    def setUp(self):
        self.owner_thread = threading.get_ident()
        self.created_on = []
        self.destroyed_on = []
        self.manager = RuntimeSensorManager(request_timeout_s=2.0)

        def create_camera():
            self.created_on.append(threading.get_ident())

            def destroy_camera():
                self.destroyed_on.append(threading.get_ident())

            return destroy_camera

        self.manager.register(
            "camera_head",
            label="头部相机",
            topic="/cam_head/color/image",
            create=create_camera,
        )

        self.readiness_polls = []

        def create_delayed_sensor():
            def ready():
                self.readiness_polls.append(threading.get_ident())
                return len(self.readiness_polls) >= 3

            return RuntimeSensorResource(destroy=lambda: None, ready=ready)

        self.manager.register(
            "camera_delayed",
            label="延迟就绪相机",
            topic="/cam_delayed/color/image",
            create=create_delayed_sensor,
        )

        self.retry_destroy_attempts = []

        def create_retry_sensor():
            def destroy():
                self.retry_destroy_attempts.append(threading.get_ident())
                if len(self.retry_destroy_attempts) == 1:
                    raise RuntimeError("temporary cleanup error")

            return destroy

        self.manager.register(
            "camera_retry",
            label="可重试相机",
            topic="/cam_retry/color/image",
            create=create_retry_sensor,
        )
        self.manager.mark_ready()
        self.server = RuntimeSensorControlServer(self.manager, port=0)
        self.server.start()

    def tearDown(self):
        self.server.stop()

    def _post_on_worker_while_isaac_updates(self, action, sensor_id="camera_head"):
        outcomes = []

        def send_request():
            request = Request(
                f"{self.server.base_url}/v1/sensors/{sensor_id}",
                data=json.dumps({"action": action}).encode("utf-8"),
                headers={"Content-Type": "application/json"},
                method="POST",
            )
            try:
                with urlopen(request, timeout=3.0) as response:
                    outcomes.append((response.status, json.loads(response.read())))
            except HTTPError as error:
                outcomes.append((error.code, json.loads(error.read())))

        worker = threading.Thread(target=send_request)
        worker.start()
        deadline = time.monotonic() + 2.0
        while worker.is_alive() and time.monotonic() < deadline:
            self.manager.process_pending()
            time.sleep(0.005)
        worker.join(timeout=0.1)
        self.assertFalse(worker.is_alive(), "sensor lifecycle request did not complete")
        self.assertEqual(len(outcomes), 1)
        return outcomes[0]

    def test_http_create_and_destroy_manage_sensor_resources_on_isaac_thread(self):
        with urlopen(f"{self.server.base_url}/v1/sensors", timeout=2.0) as response:
            initial = json.loads(response.read())
        self.assertEqual(initial["sensors"]["camera_head"]["state"], "inactive")
        self.assertFalse(initial["sensors"]["camera_head"]["active"])

        status, created = self._post_on_worker_while_isaac_updates("create")
        self.assertEqual(status, 200)
        self.assertEqual(created["sensors"]["camera_head"]["state"], "active")
        self.assertEqual(self.created_on, [self.owner_thread])
        self.assertEqual(self.destroyed_on, [])

        status, destroyed = self._post_on_worker_while_isaac_updates("destroy")
        self.assertEqual(status, 200)
        self.assertEqual(destroyed["sensors"]["camera_head"]["state"], "inactive")
        self.assertEqual(self.destroyed_on, [self.owner_thread])

    def test_repeated_create_is_idempotent_and_does_not_duplicate_resources(self):
        self.assertEqual(
            self._post_on_worker_while_isaac_updates("create", "camera_retry")[0], 200
        )
        self.assertEqual(self._post_on_worker_while_isaac_updates("create")[0], 200)
        self.assertEqual(self.created_on, [self.owner_thread])

    def test_create_waits_until_runtime_resource_reports_ready(self):
        status, response = self._post_on_worker_while_isaac_updates(
            "create", "camera_delayed"
        )

        self.assertEqual(status, 200)
        self.assertEqual(response["sensors"]["camera_delayed"]["state"], "active")
        self.assertGreaterEqual(len(self.readiness_polls), 3)
        self.assertTrue(all(
            thread_id == self.owner_thread for thread_id in self.readiness_polls
        ))

    def test_unknown_sensor_and_invalid_action_are_rejected(self):
        request = Request(
            f"{self.server.base_url}/v1/sensors/camera_missing",
            data=b'{"action":"create"}',
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        with self.assertRaises(HTTPError) as error:
            urlopen(request, timeout=2.0)
        self.assertEqual(error.exception.code, 404)

        status, payload = self._post_on_worker_while_isaac_updates("toggle")
        self.assertEqual(status, 400)
        self.assertIn("action", payload["error"])

        malformed = Request(
            f"{self.server.base_url}/v1/sensors/camera_head",
            data=b'{"action":[]}',
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        with self.assertRaises(HTTPError) as malformed_error:
            urlopen(malformed, timeout=2.0)
        self.assertEqual(malformed_error.exception.code, 400)

    def test_destroy_failure_keeps_resource_for_retry(self):
        self.assertEqual(
            self._post_on_worker_while_isaac_updates("create", "camera_retry")[0], 200
        )
        self.assertEqual(self._post_on_worker_while_isaac_updates("destroy", "camera_retry")[0], 409)
        failed = self.manager.snapshot()["sensors"]["camera_retry"]
        self.assertEqual(failed["state"], "error")
        self.assertTrue(failed["resource_present"])
        self.assertIn("temporary cleanup error", failed["error"])
        self.assertEqual(
            self._post_on_worker_while_isaac_updates("create", "camera_retry")[0], 409
        )
        self.assertEqual(len(self.retry_destroy_attempts), 1)

        status, destroyed = self._post_on_worker_while_isaac_updates("destroy", "camera_retry")
        self.assertEqual(status, 200)
        self.assertEqual(destroyed["sensors"]["camera_retry"]["state"], "inactive")
        self.assertEqual(
            self.retry_destroy_attempts, [self.owner_thread, self.owner_thread]
        )



class RobotSensorCatalogTest(unittest.TestCase):
    def test_robot_sensor_catalog_registers_inactive_factories_without_touching_stage(self):
        from pathlib import Path

        repo_root = Path(__file__).resolve().parents[2]
        stage_accesses = []
        manager = RuntimeSensorManager()
        runtime = RobotSensorRuntime(
            stage_getter=lambda: stage_accesses.append("stage requested"),
            robot_prim_path="/openflex",
            realsense_asset_dir=repo_root,
            mid360_asset_dir=repo_root,
        )

        runtime.register(manager)
        manager.mark_ready()
        snapshot = manager.snapshot()

        self.assertEqual(
            set(snapshot["sensors"]),
            {"camera_base", "camera_head", "camera_left", "camera_right", "lidar", "imu"},
        )
        self.assertEqual(snapshot["sensors"]["camera_head"]["topic"], "/cam_head/color/image")
        self.assertEqual(snapshot["sensors"]["lidar"]["topic"], "/openflex/livox_frame/lidar")
        self.assertTrue(all(
            sensor["state"] == "inactive" and not sensor["active"]
            for sensor in snapshot["sensors"].values()
        ))
        self.assertEqual(stage_accesses, [])

    def test_sensor_configuration_error_is_reported_without_faking_a_catalog(self):
        manager = RuntimeSensorManager()
        manager.mark_ready(error="missing sensor asset config")

        snapshot = manager.snapshot()

        self.assertTrue(snapshot["ready"])
        self.assertEqual(snapshot["error"], "missing sensor asset config")
        self.assertEqual(snapshot["sensors"], {})


class RuntimeSensorDeferredDestroyTest(unittest.TestCase):
    def test_destroy_stays_pending_until_runtime_reports_cleanup_complete(self):
        owner_thread = threading.get_ident()
        manager = RuntimeSensorManager(request_timeout_s=2.0)
        destroy_threads = []
        readiness_threads = []
        readiness_states = []

        def create_sensor():
            def destroy():
                destroy_threads.append(threading.get_ident())

            def destroy_ready():
                readiness_threads.append(threading.get_ident())
                readiness_states.append(manager.snapshot()["sensors"]["camera_delayed_destroy"]["state"])
                return len(readiness_threads) >= 3

            return RuntimeSensorResource(
                destroy=destroy,
                destroy_ready=destroy_ready,
            )

        manager.register(
            "camera_delayed_destroy",
            label="延迟销毁相机",
            topic="/cam_delayed_destroy/color/image",
            create=create_sensor,
        )
        manager.mark_ready()

        def request_while_processing(action):
            outcomes = []

            def request():
                outcomes.append(manager.request("camera_delayed_destroy", action))

            worker = threading.Thread(target=request)
            worker.start()
            deadline = time.monotonic() + 1.0
            while worker.is_alive() and time.monotonic() < deadline:
                manager.process_pending()
                time.sleep(0.002)
            worker.join(timeout=0.1)
            self.assertFalse(worker.is_alive(), f"{action} request did not finish")
            self.assertEqual(len(outcomes), 1)
            return outcomes[0]

        self.assertEqual(
            request_while_processing("create")["sensors"]["camera_delayed_destroy"]["state"],
            "active",
        )
        result = request_while_processing("destroy")

        self.assertEqual(result["sensors"]["camera_delayed_destroy"]["state"], "inactive")
        self.assertEqual(destroy_threads, [owner_thread])
        self.assertEqual(readiness_threads, [owner_thread] * 3)
        self.assertEqual(readiness_states, ["destroying"] * 3)


class CameraPublisherBootstrapTest(unittest.TestCase):
    def test_enable_camera_graph_before_resolving_synthetic_data_gates(self):
        events = []

        class Bridge:
            def set_camera_enabled(self, name, enabled):
                events.append(("enabled", name, enabled))

            def configure_camera_gates(self, cameras):
                events.append(("configure", cameras))

        record = {"camera_key": "camera_head", "render_product_path": "/Render/RP"}
        bootstrap = _CameraPublisherBootstrap(
            Bridge(), "camera_head", [record], warmup_updates=3
        )

        self.assertFalse(bootstrap.ready())
        self.assertFalse(bootstrap.ready())
        self.assertFalse(bootstrap.ready())
        self.assertEqual(events, [("enabled", "camera_head", True)])

        self.assertFalse(bootstrap.ready())
        self.assertFalse(bootstrap.ready())
        self.assertTrue(bootstrap.ready())
        self.assertEqual(events[0], ("enabled", "camera_head", True))
        self.assertEqual(events[1], ("configure", [record]))
        self.assertTrue(bootstrap.ready())
        self.assertEqual(len(events), 2)


if __name__ == "__main__":
    unittest.main()
