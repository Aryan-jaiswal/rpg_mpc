#!/usr/bin/env python
import cv2
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped,PointStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import signal
import sys

display = True
bridge = CvBridge()

def signal_handler(sig, frame):
	print('You pressed Ctrl+C!')
	sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)
def centeroidnp(data):

	length = data.shape[0]
	try:
		sum_x = np.sum(data[:, 0])
		sum_y = np.sum(data[:, 1])
		return sum_x/length, sum_y/length
	except:
		print("Zero Data Points")


def img_callback(img_msg):

	try:
		cv_image = bridge.imgmsg_to_cv2(img_msg,"bgr8")
	
	except CvBridgeError, e:
		rospy.logerr("CvBridge Error: {0}".format(e))
	publisher(cv_image)

def publisher(img):

	#sift = cv2.xfeatures2d.SURF_create()
	pub = rospy.Publisher('/hummingbird/mpc/point_of_interest', PointStamped, queue_size = 1)
	sift = cv2.ORB_create(nfeatures=100)
	if(display):
		cv2.namedWindow('frame', cv2.WINDOW_NORMAL)

	gray= cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
	if cv2.waitKey(1):
			
		kp = sift.detect(gray,None)
		co_ord = [kp[idx].pt for idx in range(0,len(kp))]
		x,y = centeroidnp(np.array(co_ord))
		pos = toPoseStamped(x,y)
		pub.publish(pos)
		if(display):
			cv2.circle(img,(int(x),int(y)),10,(0,0,255),-1)
			# cv2.drawKeypoints(gray,kp,img,flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
	
		if(display):
			cv2.imshow('frame',img)


def initialiser():
	
	rospy.init_node('poi_publisher',anonymous = True)
	sub_image = rospy.Subscriber("/hummingbird/camera_red_hummingbird/image_raw", Image, img_callback)
	while not rospy.is_shutdown():
		rospy.spin()
		# rate.sleep()
	if(display):
		cv2.destroyAllWindows()

def toPoseStamped(x,y):
	
	pose = PointStamped()
	pose.header.stamp = rospy.Time.now()
	pose.header.frame_id = ""
	pose.point.x = x
	pose.point.y = y
	pose.point.z = 1
	return pose


# signal.pause()
if __name__ == '__main__':
	try:
		initialiser()
	except rospy.ROSInterruptException:
		pass

# to run : export PYTHONPATH=$PYTHONPATH:/usr/lib/python2.7/dist-packages