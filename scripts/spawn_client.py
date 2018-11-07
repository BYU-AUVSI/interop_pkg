#!/usr/bin/python
import sys, rospy, rospkg

if __name__ == "__main__":
    rospy.init_node("client_spawner", anonymous=True,
        disable_signals=True)

    rospack = rospkg.RosPack()
    interop_path = rospack.get_path('interop_pkg')
    sys.path.insert(0, '%s/lib/interop/client' % (interop_path))
    # import interop
    # from auvsi_suas.client import client
    # from auvsi_suas.client import types
    from auvsi_suas.client.client import AsyncClient
    from auvsi_suas.client.types import Telemetry
    from mavlink_proxy import MavlinkProxy
    from upload_odlcs import upload_odlcs

    # client = client.Client(url='http://127.0.0.1:8000',
    #                        username='testuser',
    #                        password='testpass')
    client = AsyncClient(url='http://127.0.0.1:8000',
                         username='testuser',
                         password='testpass')

    while not rospy.is_shutdown():
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print('Kill signal received.')

    print 'Shutting down interop client...'
