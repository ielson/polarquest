#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDrive

import sys
import select
import termios
import tty
import math

MSG = """
Teleop GEM - teclado (AckermannDrive em /gem/ackermann_cmd)

Controles:
  w / s : aumenta / diminui velocidade
  a / d : esterça esquerda / direita
  x     : zera velocidade (freio suave)
  espaço: freio total (velocidade = 0, steering = 0)

  q / z : aumenta / diminui limite de velocidade

CTRL-C para sair.
"""

# w/s -> velocidade, a/d -> direção
MOVE_BINDINGS = {
    'w': (1, 0),
    's': (-1, 0),
    'a': (0, 1),
    'd': (0, -1),
}

def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def clamp(val, min_val, max_val):
    return max(min_val, min(max_val, val))


def main():
    settings = termios.tcgetattr(sys.stdin)

    rclpy.init(args=sys.argv)
    node = Node('gem_keyboard_teleop')

    pub = node.create_publisher(AckermannDrive, '/gem/ackermann_cmd', 1)

    speed      = 0.0  # m/s
    steering   = 0.0  # rad
    speed_step = 0.2
    steer_step = 0.02

    max_speed     = 10.0       # m/s (ajusta se quiser)
    max_steering  = math.radians(15.0)  # ~30 graus

    node.get_logger().info(MSG)
    rate = node.create_rate(20)  # 20 Hz

    try:
        while rclpy.ok():
            key = get_key(settings)

            if key in MOVE_BINDINGS:
                dv, dsteer = MOVE_BINDINGS[key]
                speed    += dv * speed_step
                steering += dsteer * steer_step

                speed    = clamp(speed, -max_speed, max_speed)
                steering = clamp(steering, -max_steering, max_steering)

            elif key == 'x':  # zera velocidade, mantém direção
                speed = 0.0

            elif key == ' ':  # freio total
                speed = 0.0
                steering = 0.0

            elif key == 'q':  # aumenta limite de velocidade
                max_speed += 0.5
                node.get_logger().info(f"Novo max_speed = {max_speed:.2f} m/s")

            elif key == 'z':  # diminui limite de velocidade
                max_speed = max(0.5, max_speed - 0.5)
                node.get_logger().info(f"Novo max_speed = {max_speed:.2f} m/s")

            elif key == '\x03':  # CTRL-C
                break

            # publica comando
            cmd = AckermannDrive()
            cmd.speed = speed
            cmd.steering_angle = steering
            pub.publish(cmd)

            rate.sleep()

    except Exception as e:
        node.get_logger().error(f"Erro no teleop: {e}")

    finally:
        # para o carro ao sair
        cmd = AckermannDrive()
        cmd.speed = 0.0
        cmd.steering_angle = 0.0
        pub.publish(cmd)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        rclpy.shutdown()


if __name__ == '__main__':
    main()
