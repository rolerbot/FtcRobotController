package org.firstinspires.ftc.teamcode;

public class PythonScrypts
{

    /*
    import cv2
import numpy as np
from apriltag import apriltag
import math

detector = apriltag("tag36h11")
tag_size = 0.165

calib_done = False
frame_count = 0
recommended_exposure = 2500.0
recommended_gain = 6.0

TARGET_BRIGHTNESS = 130.0

EXPOSURE_MIN = 120.0
EXPOSURE_MAX = 3300.0
GAIN_MIN = 1.0
GAIN_MAX = 6.0  # Changed from 30.0 to 6.0

def runPipeline(image, llrobot):
    global calib_done, frame_count, recommended_exposure, recommended_gain

    slider_exp = llrobot[0] if len(llrobot) > 0 else 50
    reset_trigger = llrobot[1] if len(llrobot) > 1 else 0

    offset_from_java = llrobot[2] if len(llrobot) > 2 else 0.0
    if abs(offset_from_java) < 0.001:
        offset_distance = -0.35
    else:
        offset_distance = offset_from_java

    target_tag_id = int(llrobot[3]) if len(llrobot) > 3 and llrobot[3] > 0 else 20

    if reset_trigger > 90:
        calib_done = False
        frame_count = 0
        recommended_exposure = 2500.0
        recommended_gain = 6.0

    img_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    if not calib_done:
        curr_brightness = np.mean(img_gray)

        if curr_brightness > 1:
            brightness_ratio = TARGET_BRIGHTNESS / curr_brightness

            current_total = recommended_exposure * recommended_gain
            target_total = current_total * brightness_ratio

            # First, try to achieve target with max exposure and minimum gain
            # Then increase gain only if needed, but cap at GAIN_MAX
            max_achievable = EXPOSURE_MAX * GAIN_MAX

            if target_total <= EXPOSURE_MAX * GAIN_MIN:
                # Can achieve with exposure alone at min gain
                recommended_gain = GAIN_MIN
                recommended_exposure = target_total / GAIN_MIN
                recommended_exposure = np.clip(recommended_exposure, EXPOSURE_MIN, EXPOSURE_MAX)
            elif target_total <= max_achievable:
                # Need some gain, use max exposure first
                recommended_exposure = EXPOSURE_MAX
                recommended_gain = target_total / EXPOSURE_MAX
                recommended_gain = np.clip(recommended_gain, GAIN_MIN, GAIN_MAX)
            else:
                # Too dark - max out both but respect limits
                recommended_exposure = EXPOSURE_MAX
                recommended_gain = GAIN_MAX

        frame_count += 1
        if frame_count > 40:
            calib_done = True

    detections = detector.detect(img_gray)
    llpython = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    largestContour = np.array([[]])

    if len(detections) > 0:
        target = next((d for d in detections if d['id'] == target_tag_id), None)
        if target:
            corners = target['lb-rb-rt-lt']
            corners_int = np.array(corners, dtype=np.int32)
            cv2.polylines(image, [corners_int], True, (0, 255, 0), 2)
            largestContour = corners_int.reshape(-1, 1, 2)

            obj_pts = np.array([
                [-tag_size/2, tag_size/2, 0],
                [tag_size/2, tag_size/2, 0],
                [tag_size/2, -tag_size/2, 0],
                [-tag_size/2, -tag_size/2, 0]
            ], dtype=np.float32)
            img_pts = np.array([corners[3], corners[2], corners[1], corners[0]], dtype=np.float32)
            cam_mat = np.array([
                [600, 0, image.shape[1]/2],
                [0, 600, image.shape[0]/2],
                [0, 0, 1]
            ], dtype=np.float32)

            success, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, cam_mat, np.zeros((4,1)))
            if success:
                R, _ = cv2.Rodrigues(rvec)
                target_3d = tvec.flatten() + R[:, 2] * offset_distance
                angle = math.degrees(math.atan2(target_3d[0], target_3d[2]))
                dist = float(np.linalg.norm(target_3d))

                llpython = [1.0, angle, dist, float(np.linalg.norm(tvec)), float(target_tag_id), recommended_exposure, recommended_gain, offset_distance]

                t_2d, _ = cv2.projectPoints(target_3d.reshape(1,3), np.zeros(3), np.zeros(3), cam_mat, np.zeros((4,1)))
                tx, ty = int(t_2d[0][0][0]), int(t_2d[0][0][1])
                cv2.circle(image, (tx, ty), 10, (255, 0, 255), -1)

    cv2.putText(image, f"Offset: {offset_distance:.2f}m", (10, 105), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
    cv2.putText(image, f"Target Tag: {target_tag_id}", (10, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
    cv2.putText(image, f"Rec Exposure: {int(recommended_exposure)}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
    cv2.putText(image, f"Rec Gain: {recommended_gain:.1f}", (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0) if calib_done else (0, 0, 255), 2)

    if recommended_gain >= GAIN_MAX and recommended_exposure >= EXPOSURE_MAX:
        cv2.putText(image, "WARNING: TOO DARK!", (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

    return largestContour, image, llpython
     */
}
