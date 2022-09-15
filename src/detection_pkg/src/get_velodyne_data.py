import numpy as np
import rospy
# import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
import ros_numpy


def callback(data):
    N = data.width #changed to pointcloud width
    pc = ros_numpy.numpify(data)
    points = np.zeros((N, 4)).astype(np.float32)
    points[:,0] = pc['x']
    points[:,1] = pc['y']
    points[:,2] = pc['z']
    points[:,3] = pc['intensity']


    print(points)

if __name__ == '__main__':
    rospy.init_node("velodyne_points_sub", anonymous=True)
    rospy.Subscriber('/velodyne_points', PointCloud2, callback)
    rospy.spin()