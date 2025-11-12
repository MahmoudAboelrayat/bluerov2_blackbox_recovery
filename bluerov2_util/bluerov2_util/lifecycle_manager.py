import rclpy
from rclpy.node import Node
from lifecycle_msgs.msg import Transition, State
from lifecycle_msgs.srv import ChangeState, GetState
from typing import Optional, Callable

class LifecycleManager:
    """Simplified Lifecycle Manager: configure once, then activate/deactivate."""

    def __init__(self, node: Node, node_name: str):
        self.node = node
        self.node_name = node_name
        self._logger = node.get_logger()
        self._current_state = State.PRIMARY_STATE_UNKNOWN

        self._change_state_client = None
        self._get_state_client = None
        self._create_clients()

    def _get_service_name(self, service_type: str) -> str:
        ns = self.node.get_namespace().strip('/')
        if ns:
            return f'/{ns}/{self.node_name}/{service_type}'
        return f'/{self.node_name}/{service_type}'

    def _create_clients(self) -> bool:
        """Create lifecycle service clients safely."""
        try:
            self._change_state_client = self.node.create_client(ChangeState, self._get_service_name('change_state'))
            self._get_state_client = self.node.create_client(GetState, self._get_service_name('get_state'))

            if not self._change_state_client.wait_for_service(timeout_sec=1.0):
                self._logger.warn(f'Lifecycle services for {self.node_name} not available (timeout).')
                return False

            self.get_state_sync()
            return True

        except Exception as e:
            self._logger.error(f"Failed to create lifecycle clients for {self.node_name}: {str(e)}")
            return False

    def configure(self) -> bool:
        """Configure the node if needed (idempotent)."""
        current_state = self.get_state_sync()
        if current_state in [State.PRIMARY_STATE_INACTIVE, State.PRIMARY_STATE_ACTIVE]:
            self._logger.info(f"{self.node_name} is already configured ({self.get_state_name(current_state)}).")
            return True

        if current_state not in [State.PRIMARY_STATE_UNCONFIGURED, State.PRIMARY_STATE_FINALIZED]:
            self._logger.warn(f"{self.node_name} cannot be configured from state {self.get_state_name(current_state)}.")
            return False

        self._logger.info(f"Configuring {self.node_name}...")
        req = ChangeState.Request()
        req.transition.id = Transition.TRANSITION_CONFIGURE
        future = self._change_state_client.call_async(req)

        rclpy.spin_until_future_complete(self.node, future, timeout_sec=1.0)

        if not future.done():
            self._logger.warn(f"Configuration timeout for {self.node_name}")
            return False

        result = future.result()
        if result and result.success:
            self._current_state = State.PRIMARY_STATE_INACTIVE
            self._logger.info(f"{self.node_name} configured successfully.")
            return True

        self._logger.error(f"Failed to configure {self.node_name}")
        return False

    def activate(self, callback: Optional[Callable] = None) -> bool:
        """Activate the node if inactive. Safe: does nothing if already active."""
        current_state = self.get_state_sync()

        if current_state == State.PRIMARY_STATE_ACTIVE:
            self._logger.info(f"{self.node_name} already active. No action taken.")
            if callback:
                self._call_success_callback(callback)
            return True

        if current_state != State.PRIMARY_STATE_INACTIVE:
            self._logger.warn(f"{self.node_name} cannot be activated from state {self.get_state_name(current_state)}.")
            return False

        self._logger.info(f"Activating {self.node_name}...")
        req = ChangeState.Request()
        req.transition.id = Transition.TRANSITION_ACTIVATE
        future = self._change_state_client.call_async(req)
        future.add_done_callback(callback or self._default_activation_callback)
        return True


    def deactivate(self, callback: Optional[Callable] = None) -> bool:
        """Deactivate the node if active. Safe: does nothing if already inactive."""
        current_state = self.get_state_sync()

        if current_state == State.PRIMARY_STATE_INACTIVE:
            self._logger.info(f"{self.node_name} already inactive. No action taken.")
            if callback:
                self._call_success_callback(callback)
            return True

        if current_state != State.PRIMARY_STATE_ACTIVE:
            self._logger.warn(f"{self.node_name} cannot be deactivated from state {self.get_state_name(current_state)}.")
            return False

        self._logger.info(f"Deactivating {self.node_name}...")
        req = ChangeState.Request()
        req.transition.id = Transition.TRANSITION_DEACTIVATE
        future = self._change_state_client.call_async(req)
        future.add_done_callback(callback or self._default_deactivation_callback)
        return True


    def get_state_sync(self) -> int:
        """Get current node state safely."""
        if not self._get_state_client:
            return State.PRIMARY_STATE_UNKNOWN
        try:
            future = self._get_state_client.call_async(GetState.Request())
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=0.5)
            if future.done() and future.result():
                self._current_state = future.result().current_state.id
            return self._current_state
        except Exception as e:
            self._logger.warn(f"Failed to get state of {self.node_name}: {str(e)}")
            return State.PRIMARY_STATE_UNKNOWN

    def get_state_name(self, state_id: int) -> str:
        names = {
            State.PRIMARY_STATE_UNCONFIGURED: "UNCONFIGURED",
            State.PRIMARY_STATE_INACTIVE: "INACTIVE",
            State.PRIMARY_STATE_ACTIVE: "ACTIVE",
            State.PRIMARY_STATE_FINALIZED: "FINALIZED",
            State.PRIMARY_STATE_UNKNOWN: "UNKNOWN"
        }
        return names.get(state_id, f"UNKNOWN({state_id})")

    def _call_success_callback(self, callback: Callable):
        class MockFuture:
            def result(self):
                class MockResult:
                    success = True
                return MockResult()
        callback(MockFuture())

    def _default_activation_callback(self, future):
        try:
            res = future.result()
            if res and res.success:
                self._current_state = State.PRIMARY_STATE_ACTIVE
                self._logger.info(f"{self.node_name} activated successfully.")
            else:
                self._logger.error(f"{self.node_name} activation failed.")
        except Exception as e:
            self._logger.error(f"Exception in activation callback: {str(e)}")

    def _default_deactivation_callback(self, future):
        try:
            res = future.result()
            if res and res.success:
                self._current_state = State.PRIMARY_STATE_INACTIVE
                self._logger.info(f"{self.node_name} deactivated successfully.")
            else:
                self._logger.error(f"{self.node_name} deactivation failed.")
        except Exception as e:
            self._logger.error(f"Exception in deactivation callback: {str(e)}")
