#!/usr/bin/python
# from __future__ import print_function # Only Python 2.x
# import rospy, rospkg, subprocess, shlex
import rospy, rospkg, shlex, os
from subprocess import Popen, PIPE
from threading import Timer

# def execute(cmd):
#     command = shlex.split(cmd)
#     print 'before'
#     p = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
#         universal_newlines=True)
#     print 'after'
#     output = ""
#     while p.poll() is None:
#         print 'loop'
#         l = p.stdout.readline() # This blocks until it receives a newline.
#         output = output + l
#     output = output + p.stdout.read()
#     print output.strip()
    # print p.stdout.read()
    # output, err = popen.communicate()
    # print(output)
    # print(err)
    # for stdout_line in iter(popen.stdout.readline, ""):
    #     yield stdout_line
    # popen.stdout.close()
    # return_code = popen.wait()
    # if return_code:
    #     raise subprocess.CalledProcessError(return_code, cmd)

# def kill_proc(proc):
#     os.system('sudo kill %s' % (proc.pid))
class KillableProcess(object):
    def __init__(self, proc):
        self.proc_ = proc
        print self.proc_.pid
    def kill(self):
        print 'killing'
        os.system('sudo kill -9 %s' % (self.proc_.pid))
        print 'killed'
    def getProc(self):
        return self.proc_

def run(cmd, timeout_sec):
    os.system('echo_blue hello')
    proc = Popen(shlex.split(cmd), stdout=PIPE, stderr=PIPE)
    print proc.pid
    KP = KillableProcess(proc)
    # timer = Timer(timeout_sec, proc.kill)
    timer = Timer(timeout_sec, KP.kill)
    try:
        timer.start()
        stdout, stderr = KP.getProc().communicate()
    finally:
        timer.cancel()

if __name__ == "__main__":
    rospy.init_node("interop_server_spawner", anonymous=True,
        disable_signals=True)

    rospack = rospkg.RosPack()
    interop_pkg_path = rospack.get_path('interop_pkg')

    # output = subprocess.check_output(["sudo","%s/lib/interop/server/run.sh" % (interop_pkg_path)])
    # run("sudo %s/lib/interop/server/run.sh" % (interop_pkg_path), 3)
    # os.system("sudo %s/lib/interop/server/run.sh" % (interop_pkg_path))
    os.system('sudo docker start interop-server')
    print 'umm'
    # if 'Error' in output:
    #     print 'ERROR'

    # print '...',output

    # try:
    #     print('about to try the process...')
    #     output = subprocess.check_output(["sudo","%s/lib/interop/server/run.sh" % (interop_pkg_path)])
    #
    #     # execute("sudo %s/lib/interop/server/run.sh" % (interop_pkg_path))
    #     # for path in execute(["ls"]):
    #     #     print(path, end="")
    #     # execute("echo_blue 'ummmm'")
    #     print('Process running')
    # except:
    #     print('Docker already installed!')
    print('done running')
    while not rospy.is_shutdown():
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print('Shutting down the interop server')
            os.system('sudo docker stop interop-server')
    print 'actually shutting down interop server'
    os.system('sudo docker stop interop-server')
