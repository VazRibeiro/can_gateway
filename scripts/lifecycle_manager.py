#!/usr/bin/env python3
"""Very small lifecycle manager: auto-configure+activate a list of nodes."""
import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import Transition
import time

class LifecycleManager(Node):
    def __init__(self):
        super().__init__('lifecycle_manager')
        self.declare_parameter('managed_nodes', ['can_gateway'])
        self.nodes = self.get_parameter('managed_nodes').get_parameter_value().string_array_value
        self.get_logger().info(f'Managing nodes: {self.nodes}')
        # Use a longer initial delay to let nodes start up
        self.timer = self.create_timer(5.0, self.manage)
        self.step = 0
        self.max_retries = 3
        self.retry_count = 0

    def manage(self):
        try:
            if self.step == 0:
                self.get_logger().info('Checking and configuring nodes…')
                success = True
                for n in self.nodes:
                    # First check current state
                    current_state = self._get_state(n)
                    self.get_logger().info(f'{n} current state: {current_state}')
                    
                    # Only configure if in unconfigured state
                    if current_state == 'unconfigured':
                        if not self._change_state(n, Transition.TRANSITION_CONFIGURE):
                            success = False
                            break
                    elif current_state == 'inactive':
                        self.get_logger().info(f'{n} already configured, skipping to activation')
                    else:
                        self.get_logger().warning(f'{n} in unexpected state: {current_state}')
                        
                if success:
                    self.step = 1
                    self.retry_count = 0
                else:
                    self._handle_failure()
                    
            elif self.step == 1:
                self.get_logger().info('Activating nodes…')
                success = True
                for n in self.nodes:
                    current_state = self._get_state(n)
                    self.get_logger().info(f'{n} current state before activation: {current_state}')
                    
                    if current_state == 'inactive':
                        if not self._change_state(n, Transition.TRANSITION_ACTIVATE):
                            success = False
                            break
                    elif current_state == 'active':
                        self.get_logger().info(f'{n} already active')
                    else:
                        self.get_logger().warning(f'{n} not in inactive state, cannot activate')
                        
                if success:
                    self.step = 2
                    self.get_logger().info('All nodes activated successfully!')
                    self.timer.cancel()
                else:
                    self._handle_failure()
        except Exception as e:
            self.get_logger().error(f'Exception in manage: {e}')
            self._handle_failure()

    def _handle_failure(self):
        self.retry_count += 1
        if self.retry_count >= self.max_retries:
            self.get_logger().error(f'Failed after {self.max_retries} retries, giving up')
            self.timer.cancel()
        else:
            self.get_logger().warning(f'Retry {self.retry_count}/{self.max_retries} in 5 seconds...')
            # Reset step to try again
            self.step = 0

    def _get_state(self, target):
        """Get the current lifecycle state of a node."""
        try:
            cli = self.create_client(GetState, f'{target}/get_state')
            if not cli.wait_for_service(timeout_sec=5.0):
                self.get_logger().warning(f'get_state service not available for {target}')
                return 'unknown'
                
            req = GetState.Request()
            fut = cli.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
            
            if fut.done():
                result = fut.result()
                if result:
                    state_id = result.current_state.id
                    # Map state IDs to readable names
                    state_names = {
                        0: 'unknown',
                        1: 'unconfigured', 
                        2: 'inactive',
                        3: 'active',
                        4: 'finalized'
                    }
                    return state_names.get(state_id, f'unknown_state_{state_id}')
                else:
                    return 'unknown'
            else:
                self.get_logger().error(f'{target}: get_state → TIMEOUT')
                return 'unknown'
                
        except Exception as e:
            self.get_logger().error(f'Exception getting state for {target}: {e}')
            return 'unknown'

    def _change_state(self, target, transition_id):
        try:
            cli = self.create_client(ChangeState, f'{target}/change_state')
            self.get_logger().info(f'Waiting for service {target}/change_state...')
            
            if not cli.wait_for_service(timeout_sec=10.0):
                self.get_logger().warning(f'Service not available for {target} after 10s timeout')
                return False
                
            req = ChangeState.Request()
            req.transition.id = transition_id
            self.get_logger().info(f'Calling transition {transition_id} on {target}')
            
            fut = cli.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
            
            if fut.done():
                result = fut.result()
                if result and result.success:
                    self.get_logger().info(f'{target}: transition {transition_id} → SUCCESS')
                    return True
                else:
                    self.get_logger().error(f'{target}: transition {transition_id} → FAILED')
                    return False
            else:
                self.get_logger().error(f'{target}: transition {transition_id} → TIMEOUT')
                return False
                
        except Exception as e:
            self.get_logger().error(f'Exception calling service for {target}: {e}')
            return False


def main():
    rclpy.init()
    node = LifecycleManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()