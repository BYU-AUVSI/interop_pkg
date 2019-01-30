#!/usr/bin/python
import rospy, rospkg, shlex, os
from subprocess import Popen, PIPE
from threading import Timer

if __name__ == "__main__":
    rospy.init_node("interop_server_spawner", anonymous=True,
        disable_signals=True)

    os.system('docker start interop-server')
    print('SERVER RUNNING')

    while not rospy.is_shutdown():
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print('Kill signal received.')

    print('Shutting down interop server...')
    os.system('docker stop interop-server')
