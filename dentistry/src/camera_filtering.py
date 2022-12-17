import argparse
import numpy as np
import os

import cv2
from cv_bridge import CvBridge, CvBridgeError

import rospy
import intera_interface
import itertools

atomic_counter = itertools.count()

def vision_processing(cv_image):
    first_img = cv_image

    orig_0 = cv_image.shape[0]
    orig_1 = cv_image.shape[1]

    cv_image2 = cv_image

    cv_image = cv_image[int(cv_image.shape[0]/2) + 90 :cv_image.shape[0] - 200]
    cv_image = cv_image[:, 400:cv_image.shape[1] - 380]

    cv_image2 = cv_image2[int(cv_image2.shape[0]/2) + 90 :cv_image2.shape[0] - 200]
    cv_image2 = cv_image2[:, 500:cv_image2.shape[1] - 480]

    orig_img = cv_image
    orig_img2 = cv_image2

    cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    cv_image2 = cv2.cvtColor(cv_image2, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(cv_image, (0, 0, 0), (180, 255, 30))
    mask_mouth = cv2.inRange(cv_image2, (0, 0, 100), (40, 200, 255))

    # cv2.floodFill(mask_mouth, mask_mouth, (0, int(cv_image2.shape[1]/2)),255)

    contours,_ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    contours_mouth,_ = cv2.findContours(mask_mouth, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)


    if len(contours) != 0:
        # the contours are drawn here
        cv2.drawContours(orig_img, contours, -1, 255, 3)
        cv2.drawContours(orig_img2, contours_mouth, -1, 255, 3)

        #find the biggest area of the contour
        c = max(contours, key = cv2.contourArea)
        c_mouth = max(contours_mouth, key = cv2.contourArea)

        x,y,w,h = cv2.boundingRect(c)
        x2,y2,w2,h2 = cv2.boundingRect(c_mouth)
        # draw the 'human' contour (in green)
        cv2.rectangle(orig_img,(x,y),(x+w,y+h),(0,255,0),2)
        # cv2.rectangle(orig_img2,(x,y),(x+w,y+h),(0,255,0),2)

    M = cv2.moments(c)
    centroid_x = int(M['m10']//M['m00'])
    centroid_y = int(M['m01']//M['m00'])

    M2 = cv2.moments(c_mouth)
    centroid_x2 = int(M2['m10']//M2['m00'])
    centroid_y2 = int(M2['m01']//M2['m00'])

    cy_true = int(orig_0/2) + 90 + centroid_y
    cx_true = 400 + centroid_x

    cy2_true = int(orig_0/2) + 90 + centroid_y2
    cx2_true = 400 + centroid_x2

    cv2.rectangle(first_img, (cx_true - int(w/2), cy_true - int(h/2)), (cx_true + int(w/2), cy_true + int(h/2)), (0, 255, 0), 2)
    cv2.rectangle(first_img, (cx2_true - int(w2/2), cy2_true - int(h2/2)), (cx2_true + int(w2/2), cy2_true + int(h2/2)), (0, 255, 0), 2)

    print('Getting tooth centroid: ', (cx_true, cy_true))
    return (cx_true, cy_true)
    # cv2.namedWindow("cv_image", 0)
    # refresh the image on the screen
    # cv2.imshow("cv_image", first_img)
    # cv2.waitKey(3)

def show_image_callback(img_data, xxx_todo_changeme):
    """The callback function to show image by using CvBridge and cv
    """

    cameras = intera_interface.Cameras()
    curr_count = next(atomic_counter)
    print('Callback #' + str(curr_count) + ":")

    (edge_detection, window_name) = xxx_todo_changeme
    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(img_data, "bgr8")
    except CvBridgeError as err:
        rospy.logerr(err)
        return
    
    print('Starting Image Processing')
    tooth_centroid = vision_processing(cv_image)
    if cameras.is_camera_streaming('head_camera'):
        cameras.stop_streaming('head_camera')
        #demo.main(tooth_centroid)
        os.system('python3 demo.py ' + str(tooth_centroid[0]) + ' ' + str(tooth_centroid[1]))

def main():
    """Camera Display Example
    Cognex Hand Camera Ranges
        - exposure: [0.01-100]
        - gain: [0-255]
    Head Camera Ranges:
        - exposure: [0-100], -1 for auto-exposure
        - gain: [0-79], -1 for auto-gain
    """
    rp = intera_interface.RobotParams()
    valid_cameras = rp.get_camera_names()
    if not valid_cameras:
        rp.log_message(("Cannot detect any camera_config"
            " parameters on this robot. Exiting."), "ERROR")
        return
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    parser.add_argument(
        '-c', '--camera', type=str, default="head_camera",
        choices=valid_cameras, help='Setup Camera Name for Camera Display')
    parser.add_argument(
        '-r', '--raw', action='store_true',
        help='Specify use of the raw image (unrectified) topic')
    parser.add_argument(
        '-e', '--edge', action='store_true',
        help='Streaming the Canny edge detection image')
    parser.add_argument(
        '-g', '--gain', type=int,
        help='Set gain for camera (-1 = auto)')
    parser.add_argument(
        '-x', '--exposure', type=float,
        help='Set exposure for camera (-1 = auto)')
    args = parser.parse_args(rospy.myargv()[1:])

    print("Initializing node... ")
    rospy.init_node('camera_display', anonymous=True)
    cameras = intera_interface.Cameras()
    if not cameras.verify_camera_exists(args.camera):
        rospy.logerr("Could not detect the specified camera, exiting the example.")
        return
    rospy.loginfo("Opening camera '{0}'...".format(args.camera))
    cameras.start_streaming(args.camera)
    rectify_image = not args.raw
    use_canny_edge = args.edge
    cameras.set_callback(args.camera, show_image_callback,
        rectify_image=rectify_image, callback_args=(use_canny_edge, args.camera))

    # optionally set gain and exposure parameters
    if args.gain is not None:
        if cameras.set_gain(args.camera, args.gain):
            rospy.loginfo("Gain set to: {0}".format(cameras.get_gain(args.camera)))

    if args.exposure is not None:
        if cameras.set_exposure(args.camera, args.exposure):
            rospy.loginfo("Exposure set to: {0}".format(cameras.get_exposure(args.camera)))

    def clean_shutdown():
        print("Shutting down camera_display node.")
        cv2.destroyAllWindows()

    rospy.on_shutdown(clean_shutdown)
    rospy.loginfo("Camera_display node running. Ctrl-c to quit")
    rospy.spin()


if __name__ == '__main__':
    main()