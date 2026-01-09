#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import pyrealsense2 as rs
import numpy as np
import cv2
import cv2.aruco as aruco
import time
from xarm_msgs.msg import XyzCoor
from xarm_msgs.msg import MarkerID_XyzCoor
from yolov5 import detect
import json

rospy.init_node("Detect", anonymous=True)
pipeline = rs.pipeline()  # 定义流程pipeline
config = rs.config()  # 定义配置config
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
profile = pipeline.start(config)  # 流程开始
align_to = rs.stream.color  # 与color流对齐
align = rs.align(align_to)


def get_aligned_images():
    frames = pipeline.wait_for_frames()  # 等待获取图像帧
    aligned_frames = align.process(frames)  # 获取对齐帧
    aligned_depth_frame = aligned_frames.get_depth_frame()  # 获取对齐帧中的depth帧
    color_frame = aligned_frames.get_color_frame()  # 获取对齐帧中的color帧

    ############### 相机参数的获取 #######################
    intr = color_frame.profile.as_video_stream_profile().intrinsics  # 获取相机内参
    depth_intrin = aligned_depth_frame.profile.as_video_stream_profile().intrinsics  # 获取深度参数（像素坐标系转相机坐标系会用到）

    # 相机内参转换为np.array
    intr_matrix = np.array([
        [intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1]
    ])

    depth_image = np.asanyarray(aligned_depth_frame.get_data())  # 深度图（默认16位）
    depth_image_8bit = cv2.convertScaleAbs(depth_image, alpha=0.03)  # 深度图（8位）
    depth_image_3d = np.dstack(
        (depth_image_8bit, depth_image_8bit, depth_image_8bit))  # 3通道深度图
    color_image = np.asanyarray(color_frame.get_data())  # RGB图
    camera_parameters = {'fx': intr.fx, 'fy': intr.fy,
                         'ppx': intr.ppx, 'ppy': intr.ppy,
                         'height': intr.height, 'width': intr.width,
                         'depth_scale': profile.get_device().first_depth_sensor().get_depth_scale()
                         }

    # 保存内参到本地
    with open('./intrinsics.json', 'w') as fp:
        json.dump(camera_parameters, fp)

    print(f"intr: {intr}\nintr_matrix:{intr_matrix}\n "
          f"depth_intrin: {depth_intrin}\n intr.coeffs: {np.array(intr.coeffs)}\n")
    #######################################################
    # 返回相机内参、相机内参的np、深度参数、彩色图、深度图、齐帧中的depth帧、相机畸变系数的np
    return (intr, intr_matrix, depth_intrin, color_image,
            depth_image, aligned_depth_frame, np.array(intr.coeffs))


def detect_aruco(color_img, intr_matrix, intr_coeffs):
    # 获取dictionary，4*4码，指示位50个
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    # 创建detector parameters
    parameters = aruco.DetectorParameters()
    # 输入rgb，dictionary，相机内参，相机畸变系数
    detector = aruco.ArucoDetector(aruco_dict, parameters)
    corners, ids, rejected_img_points = aruco.detectMarkers(color_img,
                                                            aruco_dict, parameters=parameters)
    # rvec是旋转向量，tvec是平移向量
    rvec, tvec, markerPoints = aruco.estimatePoseSingleMarkers(corners,
                                                               0.045, intr_matrix,
                                                               intr_coeffs)
    # test
    """
    print("rvec:")
    print(rvec)
    print("tvec:")
    print(tvec)
    print("corners:")
    print(corners)
    """
    img_temp = color_img
    if corners is None or ids is None:
        return img_temp, None, None

    # 计算四个点的中心
    centers = []
    # print("corners:", corners)
    for corner in corners:
        for a_corner in corner:
            # corner为[[],[],[],[]]
            # print("corner:", corner)
            centerx = 0
            centery = 0
            for each_corner in a_corner:
                # each_corner为[ , ]
                # print("each corner:", each_corner)
                centerx += each_corner[0]
                centery += each_corner[1]
            centerx /= 4
            centery /= 4
            centerx = int(centerx)
            centery = int(centery)
            centers.append([centerx, centery])
    # print("centers:", centers)
    # 标出aruco码的位置

    real_ids = []
    # ids is [[1] [2] [3]]
    for each_id in ids:
        # each_id is [1]
        for single_each_id in each_id:
            # single_each_id is 1
            real_ids.append(single_each_id)

    if corners is not None and ids is not None:
        # aruco.drawDetectedMarkers(img_temp, corners)
        # 画xyz轴
        # cv2.drawFrameAxes(img_temp, intr_matrix, intr_coeffs, rvec, tvec, 0.05)
        # cv2.imshow('image', color_img)
        # print(ids)
        # print(centers)
        # print real_ids
        return img_temp, centers, real_ids


def droplet_talker(topic_name, x, y, z):
    pub = rospy.Publisher(topic_name, XyzCoor, queue_size=10)
    # rate = rospy.Rate(10)
    coor = XyzCoor()
    coor.x = x
    coor.y = y
    coor.z = z
    if coor.x is None:
        rospy.loginfo(f"No {topic_name},\n{coor}")
        pub.publish(coor)
    else:
        rospy.loginfo(f"Publish {topic_name}:\n{coor}")
    pub.publish(coor)
    """
        while not rospy.is_shutdown():
        rospy.loginfo("Publish droplet coordinate:",coor)
        pub.publish(coor)
        rate.sleep()
    """


def marker_talker(topic_name, _id, x, y, z):
    pub = rospy.Publisher(topic_name, MarkerID_XyzCoor, queue_size=10)
    # rate = rospy.Rate(10)
    coor = MarkerID_XyzCoor()
    coor.id = _id
    coor.x = x
    coor.y = y
    coor.z = z
    if coor.x is None:
        rospy.loginfo(f"No {topic_name},\n{coor}")
        pub.publish(coor)
    else:
        rospy.loginfo(f"Publish {topic_name}:\n{coor}")
    pub.publish(coor)
    """
        while not rospy.is_shutdown():
        rospy.loginfo("Publish droplet coordinate:",coor)
        pub.publish(coor)
        rate.sleep()
    """


def camera_adjust(coor):
    temp_z = coor[1]
    temp_x = coor[2]
    temp_y = coor[0]
    return [temp_x, temp_y, temp_z]


def select_best_bbox(bbox, confidence):
    if confidence is None:
        # print(1)
        best_bbox = None
        best_confidence = None
    elif len(confidence) >= 2:
        # print(2)
        best_index1 = np.argmax(confidences)
        best_conf1 = confidence[best_index1]
        temp_conf = confidences
        temp_conf[best_index1] = -np.inf
        best_index2 = np.argmax(temp_conf)
        best_conf2 = confidence[best_index2]
        best_index = [best_index1, best_index2]
        best_bbox = bbox[best_index, :]
        best_confidence = [best_conf1, best_conf2]
    else:
        # print(3)
        best_bbox = None
        best_confidence = None
    return best_bbox, best_confidence


if __name__ == '__main__':
    weights_path = "/client_move/src/scripts/yolov5/runs/train/test011202306/weights/best.pt"
    try:
        t_droplet_start = time.time()
        rospy.loginfo(t_droplet_start)
        t_marker_start = t_droplet_start
        while not rospy.is_shutdown():

            # Wait for a coherent pair of frames: depth and color
            # 相机内参、相机内参的np、深度参数、彩色图、深度图、齐帧中的depth帧、相机畸变系数的np
            intr, intr_matrix, depth_intrin, color_image, depth_image, aligned_depth_frame, intr_coeffs = get_aligned_images()  # 获取对齐的图像与相机内参
            if not depth_image.any() or not color_image.any():
                continue
            # Convert images to numpy arrays
            # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.50), cv2.COLORMAP_JET)
            # Stack both images horizontally
            images = np.hstack((color_image, depth_colormap))

            ### detect droplets ###
            images_det, bbox, label, confidences = detect.run(weights=weights_path, source=color_image, nosave=True)
            # bbox: bounding box with [x,y,w,h] in normalization
            bbox = np.array(bbox)
            confidences = np.array(confidences)

            h, w = color_image.shape[0], color_image.shape[1]
            # select two best bbox with two highest confidences
            two_best_bbox, two_best_confidence = select_best_bbox(bbox, confidences)
            image_det_best = color_image
            # print(two_best_bbox)
            if two_best_bbox is not None:
                # convert bbox from normalization coor to pixel in image
                pixel_bbox = two_best_bbox[:, :] * np.array([[w, h, w, h], [w, h, w, h]])
                pixel_bbox = np.array(pixel_bbox, dtype=int)
                # draw two rectangles
                for single_bbox, single_confidence in zip(pixel_bbox, two_best_confidence):
                    image_det_best = cv2.rectangle(image_det_best,
                                                   (int(single_bbox[0] - single_bbox[2] / 2),
                                                    int(single_bbox[1] - single_bbox[3] / 2)),
                                                   (int(single_bbox[0] + single_bbox[2] / 2),
                                                    int(single_bbox[1] + single_bbox[3] / 2)),
                                                   (0, 0, 255), 2)
                    image_det_best = cv2.putText(image_det_best, '{:.2f}'.format(single_confidence),
                                                 (int(single_bbox[0] - single_bbox[2] / 2),
                                                  int(single_bbox[1] - single_bbox[3] / 2) - 5),
                                                 cv2.FONT_HERSHEY_SIMPLEX,
                                                 0.4, (0, 0, 255), 1)
                # calculate the middle point of the two droplets
                middle_pixel = (pixel_bbox[0, 0:2] + pixel_bbox[1, 0:2]) / 2.0
                middle_pixel = np.array(middle_pixel, dtype=int)
                # print(middle_pixel)
                middle_depth = aligned_depth_frame.get_distance(middle_pixel[0], middle_pixel[1])
                # obtain the 3D coor in camera system of the middle point
                point_camera_coor = rs.rs2_deproject_pixel_to_point(depth_intrin, middle_pixel, middle_depth)
                t_talk_droplet = time.time()
                # publish the coor of the middle point every 0.1s
                if t_talk_droplet - t_droplet_start >= 1:
                    # convert to x,y,z of correct order according to definition of camera link
                    point_camera_coor = camera_adjust(point_camera_coor)
                    droplet_talker('droplet_xyz_in_camera', point_camera_coor[0], point_camera_coor[1],
                                   point_camera_coor[2])
                    t_droplet_start = time.time()
            else:

                rospy.loginfo('No droplet detected')
                # droplet_talker('droplet_xyz_in_camera', None, None, None)

            ### detect Aruco ###
            image_with_frame, marker_centers, ids = detect_aruco(color_image, intr_matrix, intr_coeffs)
            marker_depths = []
            markers_camera_coor = []
            image_with_MarkerDroplet = image_det_best
            if ids is not None and marker_centers is not None:
                for id_, marker_center in zip(ids, marker_centers):
                    # print(id_,marker_center)
                    # add marker to the images with droplet detected
                    image_with_MarkerDroplet = cv2.circle(image_det_best,
                                                          (marker_center[0], marker_center[1]),
                                                          5, (0, 255, 0), -1)
                    image_with_MarkerDroplet = cv2.putText(image_with_MarkerDroplet,
                                                           f'ID: {id_}',
                                                           (marker_center[0] - 10, marker_center[1] - 10),
                                                           cv2.FONT_HERSHEY_SIMPLEX,
                                                           0.5, (0, 255, 0), 2)
                    marker_depth = aligned_depth_frame.get_distance(marker_center[0], marker_center[1])
                    marker_depths.append(marker_depth)
                    marker_camera_coor = rs.rs2_deproject_pixel_to_point(depth_intrin, marker_center, marker_depth)
                    markers_camera_coor.append(marker_camera_coor)
                t_talk_marker = time.time()
                if t_talk_marker - t_marker_start >= 1:
                    for single_id, marker_coor in zip(ids, markers_camera_coor):
                        if single_id == 1:
                            marker_coor = camera_adjust(marker_coor)
                            marker_talker('marker_xyz_in_camera', single_id,
                                          marker_coor[0], marker_coor[1], marker_coor[2])
                        t_marker_start = time.time()
            else:
                rospy.loginfo('No marker detected')
                # marker_talker('marker_xyz_in_camera', None, None, None, None)

            t_end = time.time()  # 结束计时
            cv2.namedWindow('detection', cv2.WINDOW_NORMAL)
            cv2.imshow('detection', image_with_MarkerDroplet)
            key = cv2.waitKey(1)
            # Press esc or 'q' to close the image window
            if key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()
                break
    except rospy.ROSInterruptException:
        pipeline.stop()
        pass
    finally:
        # Stop streaming
        pipeline.stop()
