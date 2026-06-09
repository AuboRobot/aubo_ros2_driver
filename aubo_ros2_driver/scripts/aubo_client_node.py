#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from aubo_msgs.srv import JsonRpc
import json
import websocket


class JsonRpcWebSocketService(Node):
    def __init__(self):
        super().__init__('aubo_client')

        # Declare and get parameters
        self.declare_parameter('jsonrpc.ip', '127.0.0.1')
        self.declare_parameter('jsonrpc.port', 9012)
        self.declare_parameter('jsonrpc.robot_prefix', 'rob1')

        self.server_ip = self.get_parameter('jsonrpc.ip').get_parameter_value().string_value
        self.server_port = self.get_parameter('jsonrpc.port').get_parameter_value().integer_value
        self.robot_prefix = self.get_parameter('jsonrpc.robot_prefix').get_parameter_value().string_value
        self.url = f'ws://{self.server_ip}:{self.server_port}'

        self.ws = None
        comm = self.connect_websocket()

        # Create the service
        self.srv = self.create_service(JsonRpc, 'jsonrpc_service', self.handle_service)

        if comm:
            self.get_logger().info(
                f'[AUBO CLIENT] Ready to send JSON-RPC to {self.url} '
                f'robot="{self.robot_prefix}"'
            )
        else:
            self.get_logger().error(
                f'[AUBO CLIENT] Not Ready to send JSON-RPC to {self.url} '
                f'robot="{self.robot_prefix}"'
            )
            quit()

    def build_method_name(self, cls: str, func: str) -> str:
        cls = cls.strip()
        func = func.strip()

        # List of classes that DO NOT use robot_prefix
        NO_PREFIX_CLASSES = [
            "AuboApi",
            "Math",
            "RegisterControl",
            "RobotInterface",
            "RuntimeMachine",
            "Serial",
            "Socket",
            "SyncMove",
            "SystemInfo",
        ]

        if '.' in func:
            # Example: "Math.add" → no prefix
            if any(func.startswith(f"{c}.") for c in NO_PREFIX_CLASSES):
                return func

            # Example: "rob1.RobotState.getTcpPose"
            if func.startswith(self.robot_prefix + "."):
                return func

            # For something like "RobotState.getTcpPose"
            # We treat this as a class + func format
            parts = func.split(".")
            if len(parts) == 2:
                c, f = parts
                if c in NO_PREFIX_CLASSES:
                    return f"{c}.{f}"
                return f"{self.robot_prefix}.{c}.{f}"

            # Unexpected format → keep original
            return func

        if cls:
            # If class is in no-prefix list → no prefix
            if cls in NO_PREFIX_CLASSES:
                return f"{cls}.{func}"
            return f"{self.robot_prefix}.{cls}.{func}"

        return f"{self.robot_prefix}.{func}"

    def connect_websocket(self):
        if self.ws:
            self.ws.close()
        try:
            self.ws = websocket.create_connection(self.url, timeout=5.0)
            self.get_logger().info(f'[WebSocket] Connected to {self.url}')
            return True
        except Exception as e:
            self.get_logger().error(f'[WebSocket] Failed to connect: {e}')
            self.ws = None
            return False

    def handle_service(self, request, response):
        if not self.ws:
            self.get_logger().warn("[WebSocket] No connection, attempting to reconnect...")
            self.connect_websocket()
            if not self.ws:
                response.result = ''
                response.error = "[ERROR] Failed to connect"
                return response

        # Build JSON-RPC request object
        method = self.build_method_name(request.cls, request.func)

        # Parse params (must be valid JSON string)
        try:
            params = json.loads(request.params) if request.params.strip() else []
        except Exception as e:
            self.get_logger().warn(f'[JSON] Failed to parse params, using [] instead. err={e}')
            params = []

        msg = {
            'jsonrpc': '2.0',
            'method': method,
            'params': params,
            'id': 1,
        }

        try:
            send_str = json.dumps(msg)
            self.get_logger().debug(f"[WebSocket] Sending: {send_str}")
            self.ws.send(send_str)

            data = self.ws.recv()
            self.get_logger().debug(f'[WebSocket] Received raw: {data}')

            # Try parsing JSON response
            try:
                resp_obj = json.loads(data)
            except Exception as e:
                # Response is not valid JSON
                self.get_logger().error(f'[JSON] Failed to parse response: {e}')
                response.result = ''
                response.error = data
                return response

            # JSON-RPC success case (result exists)
            if 'error' in resp_obj and resp_obj['error'] is not None:
                # JSON-RPC error case
                response.result = 'None'
                response.error = json.dumps(resp_obj['error'], ensure_ascii=False)
            else:
                # JSON-RPC had result case
                result_val = resp_obj.get('result', None)
                response.result = json.dumps(result_val, ensure_ascii=False)
                response.error = 'None'

        except Exception as e:
            self.get_logger().error(f'[WebSocket] Communication error: {e}')
            self.ws = None  # Force reconnect next time
            response.result = ''
            response.error = f'[ERROR] {e}'

        return response
    
    def destroy_node(self):
        if self.ws:
            self.ws.close()
            self.get_logger().info("[WebSocket] Connection closed.")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = JsonRpcWebSocketService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down AUBO JSON-RPC WebSocket Client Node.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
