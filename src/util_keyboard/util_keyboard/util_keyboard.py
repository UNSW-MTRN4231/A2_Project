#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pygame
import threading

class KeyboardNode(Node):
    def __init__(self):
        super().__init__('keyboard_node')
        self.publisher_ = self.create_publisher(String, 'keyboard_input', 10)
        self.subscription_ = self.create_subscription(String, 'gui_text', self.gui_text_callback, 10)
        self.last_key = None  # Store the last pressed key
        self.gui_text = "Press [space] to start operation sequence"
        self.lock = threading.Lock()

        pygame.init()
        self.get_logger().info("Keyboard node is running. Click on the window and press keys to publish.")
        pygame.display.set_mode((400, 300))  # Create a Pygame window

    def run(self):
        pygame_thread = threading.Thread(target=self.pygame_thread)
        pygame_thread.start()

        while rclpy.ok():
            rclpy.spin_once(self)  # Allow execution of ROS 2 callbacks

    def pygame_thread(self):
        while rclpy.ok():
            with self.lock:
                self.draw_window()  # Draw the Pygame window with the latest GUI text

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    return
                if event.type == pygame.KEYDOWN:
                    key_name = pygame.key.name(event.key)
                    msg = String()
                    msg.data = key_name

                    with self.lock:
                        self.publisher_.publish(msg)
                        self.last_key = key_name  # Update the last pressed key
                        self.get_logger().info(f"Published key: {key_name}")

            pygame.display.update()


    def gui_text_callback(self, msg):
        # Callback function for handling messages received on the '/gui_text' topic
        with self.lock:
            self.gui_text = msg.data
            self.get_logger().info(f"Received GUI text: {msg.data}")

    def draw_window(self):
        screen = pygame.display.get_surface()
        screen.fill((0, 0, 0))  # Clear the screen

        # Draw the last received GUI text with word wrapping
        font = pygame.font.Font(None, 36)
        wrapped_text = self.wrap_text(font, f"{self.gui_text}", 380)  # Adjust 380 based on your window size
        text_y = 50  # Starting Y position for the text

        for line in wrapped_text:
            text = font.render(line, True, (255, 255, 255))
            text_rect = text.get_rect(center=(200, text_y))
            screen.blit(text, text_rect)
            text_y += text.get_height() + 5  # Adjust 5 for spacing between lines

    def wrap_text(self, font, text, max_width):
        words = text.split(' ')
        wrapped_lines = []
        current_line = ''

        for word in words:
            test_line = current_line + word + ' '
            test_width, _ = font.size(test_line)
            if test_width <= max_width:
                current_line = test_line
            else:
                wrapped_lines.append(current_line)
                current_line = word + ' '

        wrapped_lines.append(current_line)
        return wrapped_lines


    def shutdown(self):
        pygame.quit()

def main(args=None):
    rclpy.init(args=args)
    keyboard_node = KeyboardNode()
    try:
        keyboard_node.run()
    finally:
        keyboard_node.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
