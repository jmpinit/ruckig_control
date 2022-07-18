import math
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from relaxed_ik_ros1.msg import JointAngles

current_joint_targets = [0, -math.pi / 2, math.pi / 2, 0, math.pi/2, 0]


def position(j1, j2, j3, j4, j5, j6):
    pt = JointTrajectoryPoint()
    pt.positions = [j1, j2, j3, j4, j5, j6]
    pt.velocities = [0, 0, 0, 0, 0, 0]
    pt.time_from_start = rospy.Duration(2.0 / 100)
    return pt


def solution_callback(solution):
    global current_joint_targets

    for i in range(6):
        current_joint_targets[i] = solution.angles.data[i]


def wiggle():
    global current_joint_targets

    solns_sub = rospy.Subscriber('/relaxed_ik/joint_angle_solutions', JointAngles, solution_callback)
    pub = rospy.Publisher('/ruckig_controller/command', JointTrajectory, queue_size=10)

    rospy.init_node('wiggle', anonymous=True)
    rate = rospy.Rate(100)

    start_time = rospy.get_time()

    print('Wiggling!')

    while not rospy.is_shutdown():
        now = rospy.get_time() - start_time

        cmd = JointTrajectory()
        cmd.header.stamp = rospy.Time.now()

        cmd.joint_names = ['joint_a1', 'joint_a2', 'joint_a3', 'joint_a4', 'joint_a5', 'joint_a6']
        cmd.points = [position(*current_joint_targets)]

        pub.publish(cmd)

        rate.sleep()


if __name__ == '__main__':
    try:
        wiggle()
    except rospy.ROSInterruptException:
        pass
