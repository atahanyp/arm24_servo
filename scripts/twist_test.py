#!/usr/bin/env python3
import sys, select, termios, tty, math
import rospy
from geometry_msgs.msg import TwistStamped

HELP = """
Twist Teleop (MoveIt Servo - unitless)

Linear (x,y,z):
  w/s : +x / -x
  a/d : +y / -y
  r/f : +z / -z

Angular (roll x, pitch y, yaw z):
  u/o : +roll / -roll
  i/k : +pitch / -pitch
  j/l : +yaw   / -yaw

Hız ayarı:
  =   : adım (+)
  -   : adım (-)
 
Diğer:
  space : tüm komutları sıfırla
  h     : yardım yazdır
  q     : çıkış

Not: MODE=unitless → değerler [-1,1]. YAML’de command_in_type=unitless olmalı.
"""

def get_key(timeout=0.02):
    dr, _, _ = select.select([sys.stdin], [], [], timeout)
    if dr:
        return sys.stdin.read(1)
    return None

def clamp(x, lo, hi): return max(lo, min(hi, x))

def main():
    rospy.init_node("twist_teleop")
    
    # Sabit parametreler
    topic    = "/servo_server/delta_twist_cmds"
    frame_id = "arm_base"
    rate_hz  = 100.0

    # Hız limitleri (m/s, rad/s)
    step      = 0.01   # Her tuşta artış miktarı
    lin_max   = 0.05   # Maksimum lineer hız (m/s) - 5 cm/s
    ang_max   = 0.30   # Maksimum açısal hız (rad/s) - ~17 deg/s
    decay     = 0.90   # Tuş bırakınca yavaşlama katsayısı

    pub = rospy.Publisher(topic, TwistStamped, queue_size=10)
    rate = rospy.Rate(rate_hz)

    # Mevcut hızlar
    lx=ly=lz=0.0
    ax=ay=az=0.0

    old_attr = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    print(HELP)
    print("Publishing to %s @ %.0f Hz" % (topic, rate_hz))
    print("Max: linear=%.2f m/s, angular=%.2f rad/s, step=%.3f\n" % (lin_max, ang_max, step))

    try:
        while not rospy.is_shutdown():
            key = get_key(1.0/rate_hz)

            if key:
                if   key == 'w': lx += step
                elif key == 's': lx -= step
                elif key == 'a': ly += step
                elif key == 'd': ly -= step
                elif key == 'r': lz += step
                elif key == 'f': lz -= step

                elif key == 'u': ax += step
                elif key == 'o': ax -= step
                elif key == 'i': ay += step
                elif key == 'k': ay -= step
                elif key == 'j': az += step
                elif key == 'l': az -= step

                elif key == '=': step = min(0.5, step + 0.01); print("step=%.3f" % step)
                elif key == '-': step = max(0.01, step - 0.01); print("step=%.3f" % step)

                elif key == ' ':  # stop
                    lx=ly=lz=ax=ay=az=0.0
                    print("STOP")

                elif key == 'h':
                    print(HELP.strip())

                elif key == 'q':
                    break

                # Limitlere uydur
                lx = clamp(lx, -lin_max, lin_max)
                ly = clamp(ly, -lin_max, lin_max)
                lz = clamp(lz, -lin_max, lin_max)
                ax = clamp(ax, -ang_max, ang_max)
                ay = clamp(ay, -ang_max, ang_max)
                az = clamp(az, -ang_max, ang_max)

            else:
                # tuş yoksa sönümle (yumuşak bırakma)
                lx *= decay; ly *= decay; lz *= decay
                ax *= decay; ay *= decay; az *= decay

                # çok küçükse sıfırla
                if abs(lx) < 1e-3: lx = 0.0
                if abs(ly) < 1e-3: ly = 0.0
                if abs(lz) < 1e-3: lz = 0.0
                if abs(ax) < 1e-3: ax = 0.0
                if abs(ay) < 1e-3: ay = 0.0
                if abs(az) < 1e-3: az = 0.0

            # TwistStamped yayınla (m/s & rad/s)
            msg = TwistStamped()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = frame_id
            msg.twist.linear.x  = lx
            msg.twist.linear.y  = ly
            msg.twist.linear.z  = lz
            msg.twist.angular.x = ax
            msg.twist.angular.y = ay
            msg.twist.angular.z = az

            pub.publish(msg)
            rate.sleep()

    finally:
        # çıkarken sıfır komut yayınla
        z = TwistStamped()
        z.header.stamp = rospy.Time.now()
        z.header.frame_id = frame_id
        pub.publish(z)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)
        print("\nbye.")

if __name__ == "__main__":
    main()
