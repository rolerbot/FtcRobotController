package org.firstinspires.ftc.teamcode;

public class PythonScrypts
{

    /*
    import cv2
import numpy as np
from apriltag import apriltag
import time

detector = apriltag("tag36h11")
prev_time = time.time()
fps = 0
exposure_value = 50
tag_size = 0.165
offset_distance = 0.35

def runPipeline(image, llrobot):
    global prev_time, fps, exposure_value

    current_time = time.time()
    if current_time - prev_time > 0:
        fps = 1.0 / (current_time - prev_time)
    prev_time = current_time

    img_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    mean_brightness = np.mean(img_gray)

    if mean_brightness < 80:
        exposure_value = min(exposure_value + 2, 100)
    elif mean_brightness > 180:
        exposure_value = max(exposure_value - 2, 10)

    detections = detector.detect(img_gray)
    largestContour = np.array([[]])
    llpython = [0, 0, 0, 0, 0, 0, 0, 0]

    offset_point_distance = 0

    if len(detections) > 0:
        largest_detection = None
        largest_area = 0

        for detection in detections:
            corners = detection['lb-rb-rt-lt']
            corners_np = np.array(corners, dtype=np.float32)
            area = cv2.contourArea(corners_np)
            if area > largest_area:
                largest_area = area
                largest_detection = detection

        if largest_detection is not None:
            corners = largest_detection['lb-rb-rt-lt']
            corners_int = np.array(corners, dtype=np.int32)

            cv2.polylines(image, [corners_int], True, (0, 255, 0), 3)

            center = largest_detection['center']
            cx, cy = int(center[0]), int(center[1])
            cv2.circle(image, (cx, cy), 5, (0, 255, 0), -1)

            corners_np = np.array(corners, dtype=np.float32)

            object_points = np.array([
                [-tag_size/2, -tag_size/2, 0],
                [tag_size/2, -tag_size/2, 0],
                [tag_size/2, tag_size/2, 0],
                [-tag_size/2, tag_size/2, 0]
            ], dtype=np.float32)

            image_points = np.array([
                corners[3],
                corners[2],
                corners[1],
                corners[0]
            ], dtype=np.float32)

            fx = 600
            fy = 600
            cx_cam = image.shape[1] / 2
            cy_cam = image.shape[0] / 2
            camera_matrix = np.array([
                [fx, 0, cx_cam],
                [0, fy, cy_cam],
                [0, 0, 1]
            ], dtype=np.float32)

            dist_coeffs = np.zeros((4, 1), dtype=np.float32)

            success, rvec, tvec = cv2.solvePnP(object_points, image_points, camera_matrix, dist_coeffs)

            if success:
                tag_distance = np.linalg.norm(tvec)

                R, _ = cv2.Rodrigues(rvec)
                normal = R[:, 2]
                offset_point_3d = tvec.flatten() + normal * offset_distance
                offset_point_distance = np.linalg.norm(offset_point_3d)

                offset_point_2d, _ = cv2.projectPoints(
                    offset_point_3d.reshape(1, 3),
                    np.zeros(3),
                    np.zeros(3),
                    camera_matrix,
                    dist_coeffs
                )

                ox, oy = int(offset_point_2d[0][0][0]), int(offset_point_2d[0][0][1])

                if 0 <= ox < image.shape[1] and 0 <= oy < image.shape[0]:
                    cv2.circle(image, (ox, oy), 8, (255, 0, 255), -1)
                    cv2.line(image, (cx, cy), (ox, oy), (255, 0, 255), 2)

                cv2.putText(image, f"Tag Dist: {tag_distance:.2f}m", (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.putText(image, f"Offset Dist: {offset_point_distance:.2f}m", (10, 90),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)

                largestContour = corners_int.reshape(-1, 1, 2)

                llpython = [
                    1,
                    offset_point_distance,
                    tag_distance,
                    tvec[0][0],
                    tvec[1][0],
                    tvec[2][0],
                    largest_detection['id'],
                    exposure_value
                ]

    cv2.putText(image, f"FPS: {fps:.1f}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    cv2.putText(image, f"Exp: {exposure_value}", (image.shape[1] - 100, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

    return largestContour, image, llpython
     */

    /*
    import cv2
import numpy as np
from apriltag import apriltag
import time

detector = apriltag("tag36h11")
prev_time = time.time()
fps = 0
tag_size = 0.165
offset_distance = 0.35

fx = 600
fy = 600
camera_matrix = None
dist_coeffs = np.zeros((4, 1), dtype=np.float32)

object_points = np.array([
    [-tag_size/2, tag_size/2, 0],
    [tag_size/2, tag_size/2, 0],
    [tag_size/2, -tag_size/2, 0],
    [-tag_size/2, -tag_size/2, 0]
], dtype=np.float32)

exposure_target = 120
exposure_alpha = 0.1

def runPipeline(image, llrobot):
    global prev_time, fps, camera_matrix

    current_time = time.time()
    dt = current_time - prev_time
    if dt > 0:
        fps = 1.0 / dt
    prev_time = current_time

    if camera_matrix is None:
        cx_cam = image.shape[1] / 2
        cy_cam = image.shape[0] / 2
        camera_matrix = np.array([
            [fx, 0, cx_cam],
            [0, fy, cy_cam],
            [0, 0, 1]
        ], dtype=np.float32)

    img_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    mean_brightness = np.mean(img_gray)
    if mean_brightness < exposure_target - 20:
        alpha = 1.3
        beta = 20
    elif mean_brightness > exposure_target + 20:
        alpha = 0.8
        beta = -10
    else:
        alpha = 1.0
        beta = 0

    if alpha != 1.0 or beta != 0:
        img_gray = cv2.convertScaleAbs(img_gray, alpha=alpha, beta=beta)

    img_small = cv2.resize(img_gray, (0, 0), fx=0.5, fy=0.5, interpolation=cv2.INTER_NEAREST)

    detections = detector.detect(img_small)
    largestContour = np.array([[]])
    llpython = [0, 0, 0, 0, 0, 0, 0, 0]

    if len(detections) > 0:
        largest_detection = None
        largest_area = 0

        for detection in detections:
            corners = detection['lb-rb-rt-lt']
            corners_np = np.array(corners, dtype=np.float32)
            area = cv2.contourArea(corners_np)
            if area > largest_area:
                largest_area = area
                largest_detection = detection

        if largest_detection is not None:
            corners = largest_detection['lb-rb-rt-lt']
            corners_scaled = [(c[0] * 2, c[1] * 2) for c in corners]
            corners_int = np.array(corners_scaled, dtype=np.int32)

            cv2.polylines(image, [corners_int], True, (0, 255, 0), 2)

            center = largest_detection['center']
            cx, cy = int(center[0] * 2), int(center[1] * 2)
            cv2.circle(image, (cx, cy), 4, (0, 255, 0), -1)

            lb = corners_scaled[0]
            rb = corners_scaled[1]
            rt = corners_scaled[2]
            lt = corners_scaled[3]

            image_points = np.array([
                lt,
                rt,
                rb,
                lb
            ], dtype=np.float32)

            success, rvec, tvec = cv2.solvePnP(object_points, image_points, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE)

            if success:
                tag_distance = np.linalg.norm(tvec)

                R, _ = cv2.Rodrigues(rvec)
                normal = R[:, 2]
                offset_point_3d = tvec.flatten() - normal * offset_distance

                offset_point_distance = np.linalg.norm(offset_point_3d)

                offset_point_2d, _ = cv2.projectPoints(
                    offset_point_3d.reshape(1, 3),
                    np.zeros(3),
                    np.zeros(3),
                    camera_matrix,
                    dist_coeffs
                )

                ox, oy = int(offset_point_2d[0][0][0]), int(offset_point_2d[0][0][1])

                if 0 <= ox < image.shape[1] and 0 <= oy < image.shape[0]:
                    cv2.circle(image, (ox, oy), 6, (255, 0, 255), -1)
                    cv2.line(image, (cx, cy), (ox, oy), (255, 0, 255), 2)

                cv2.putText(image, f"Tag: {tag_distance:.2f}m", (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                cv2.putText(image, f"Off: {offset_point_distance:.2f}m", (10, 85),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)

                largestContour = corners_int.reshape(-1, 1, 2)

                llpython = [
                    1,
                    offset_point_distance,
                    tag_distance,
                    tvec[0][0],
                    tvec[1][0],
                    tvec[2][0],
                    largest_detection['id'],
                    fps
                ]

    cv2.putText(image, f"FPS: {fps:.0f}", (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)
    cv2.putText(image, f"Bright: {mean_brightness:.0f}", (10, 110),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

    return largestContour, image, llpython
     */
}
