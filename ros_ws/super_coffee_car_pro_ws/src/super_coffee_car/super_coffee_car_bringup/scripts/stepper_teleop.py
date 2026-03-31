#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
import sys, select, termios, tty

msg = """
================================
  超级咖啡车 - 步进电机控制台
================================
按住 'w' 或 'W' : 步进电机上升 (+120 RPM)
按住 's' 或 'S' : 步进电机下降 (-120 RPM)
松开键盘        : 停止 (0 RPM)

按 CTRL-C 退出
================================
"""

def getKey():
    # 将终端设置为 raw 模式以读取单个字符
    tty.setraw(sys.stdin.fileno())
    # 设置 0.1 秒的超时时间，这就是“松开即停”的魔法核心
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    # 恢复终端设置
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__ == '__main__':
    # 备份终端设置
    settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('stepper_keyboard_ctrl')
    # 创建发布者，发布到你指定的 /step_motor_rpm 话题
    pub = rospy.Publisher('/step_motor_rpm', Float64, queue_size=10)

    target_rpm = 0.0
    rpm_speed = 120.0  # 你可以在这里修改默认速度

    print(msg)

    try:
        while not rospy.is_shutdown():
            key = getKey()

            if key == 'w' or key == 'W':
                target_rpm = rpm_speed
            elif key == 's' or key == 'S':
                target_rpm = -rpm_speed
            elif key == '\x03': # 捕获 CTRL-C
                break
            else:
                # 如果没按键（触发了 0.1s 超时），或者按了别的废键，速度归零
                target_rpm = 0.0

            # 发布 Float64 消息
            pub.publish(target_rpm)

    except Exception as e:
        print(e)
    finally:
        # 程序退出前，务必发一个 0，保证安全停车
        pub.publish(0.0)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)