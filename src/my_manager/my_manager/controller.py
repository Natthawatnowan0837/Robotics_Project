#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from pynput import keyboard
import threading
import time

from my_command.srv import SequenceCmd 

class KeyboardServiceNode(Node):
    def __init__(self):
        super().__init__('keyboard_service_node')
        
        self.group = ReentrantCallbackGroup()
        self.client = self.create_client(SequenceCmd, '/rotate_service', callback_group=self.group)
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok(): return
            self.get_logger().info('Wait for service...')

        self.listener = keyboard.Listener(on_press=self.on_press)
        self.listener.start()

        self.get_logger().info("Keyboard COMBO Client Ready")

    def on_press(self, key):
        try:
            if hasattr(key, 'char'):
                if key.char == 'w':
                    threading.Thread(target=self.execute_combo, args=("fwd10",)).start()
                
                elif key.char == 'q':
                    threading.Thread(target=self.execute_combo, 
                                     args=("fwd2.0","left75", "fwd3.0", "right7", "fwd8.0","right5","fwd7.0")).start()
                
                elif key.char == 'e':
                    threading.Thread(target=self.execute_combo, 
                                     args=("fwd2.2","right70", "fwd6.0", "right7","fwd8.0","left8", "fwd10.0")).start()
                    
        except Exception as e:
            self.get_logger().error(f"Error: {e}")
            
        if key == keyboard.Key.esc:
            rclpy.shutdown()

    def execute_combo(self, *commands):
        time.sleep(3.0) 

        for i, cmd in enumerate(commands):
            self.get_logger().info(f"Step {i+1}/{len(commands)}: {cmd}")
            success = self.send_request_sync(cmd)
            
            if not success:
                self.get_logger().error(f"Aborted: {cmd}")
                return

        self.get_logger().info("Finished")

    def send_request_sync(self, command_str):
        req = SequenceCmd.Request()
        req.state = command_str
        
        future = self.client.call_async(req)
        
        while not future.done():
            if not rclpy.ok(): return False
            time.sleep(0.1)
            
        try:
            response = future.result()
            return response.status == "DONE"
        except Exception as e:
            self.get_logger().error(f"Failed: {e}")
            return False

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardServiceNode()
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()