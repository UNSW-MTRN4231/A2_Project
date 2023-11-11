#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pygame

class KeyboardNode(Node):
    def __init__(self):
        super().__init__('keyboard_node')
        self.publisher_ = self.create_publisher(String, 'keyboard_input', 10)
        self.last_key = None  # Store the last pressed key

    def run(self):
        pygame.init()
        self.get_logger().info("Keyboard node is running. Click on the window and press keys to publish.")

        pygame.display.set_mode((400, 300))  # Create a Pygame window

        while rclpy.ok():
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    return
                if event.type == pygame.KEYDOWN:
                    key_name = pygame.key.name(event.key)
                    msg = String()
                    msg.data = key_name
                    self.publisher_.publish(msg)
                    self.last_key = key_name  # Update the last pressed key
                    self.get_logger().info(f"Published key: {key_name}")

            self.draw_window()  # Draw the Pygame window
            pygame.display.update()

    def draw_window(self):
        screen = pygame.display.get_surface()
        screen.fill((0, 0, 0))  # Clear the screen

        # Draw the last pressed key
        font = pygame.font.Font(None, 36)
        text = font.render(f"Last Key: {self.last_key}", True, (255, 255, 255))
        text_rect = text.get_rect(center=(200, 150))
        screen.blit(text, text_rect)

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
