import numpy as np
import rospy, cv2, cv_bridge
from sensor_msgs.msg import Image 
import pick_tooth as tp

bridge = cv_bridge.CvBridge()

def image_callback(ros_img):
	cv_image = bridge.imgmsg_to_cv2(ros_img, desired_encoding="passthrough")
	cv2.imshow("Image", cv_image)
	# Image Processing
	jaw_coords = np.array([])
	tooth_coords = np.array([])

	tp.main(tooth_coords)
	cv2.waitKey(1)

if __name__ == "__main__":
	rospy.init_node('Camera_Subscriber', anonymous=True)
	rospy.Subscriber('/cameras/head_camera/image', Image, image_callback)
	rospy.spin()
	cv2.destroyAllWindows()