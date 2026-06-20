import cv2 as cv
from pupil_apriltags import Detector
import numpy as np

# read, resize, and convert image to grayscale

cap = cv.VideoCapture(0)
detector = Detector(families = "tag36h11", nthreads = 4)

class AprilTagProcessor:
    def __init__(self):
        self.cap = cv.VideoCapture()
        
    def process_image(self):
        ret, frame = cap.read()

        # yotag = cv.resize(frame, (1440, 960))
        yotag = frame
        # print(yotag.shape)
        gray = cv.cvtColor(yotag, cv.COLOR_BGR2GRAY)

        # construct detector using the pupil_aprltags library
        fx = 1440
        fy = 960
        cx = 720
        cy = 480
        results = detector.detect(gray, estimate_tag_pose = True, camera_params = (fx, fy, cx, cy), tag_size=0.1)

        return frame, results

    def detect_tags(self, frame, results):
        distance = np.zeros(len(results))
        count = 0
        for r in results:
            # read corners from results and convert them from floats to ints
            (ptA, ptB, ptC, ptD) = r.corners
            ptA = (int(ptA[0]), int(ptA[1]))
            ptB = (int(ptB[0]), int(ptB[1]))
            ptC = (int(ptC[0]), int(ptC[1]))
            ptD = (int(ptD[0]), int(ptD[1]))

            # draw bounding boxes for apriltags
            cv.line(frame, ptA, ptB, (0, 255, 0), 2)
            cv.line(frame, ptB, ptC, (0, 255, 0), 2)
            cv.line(frame, ptC, ptD, (0, 255, 0), 2)
            cv.line(frame, ptD, ptA, (0, 255, 0), 2)

            # mark the (x,y) center of the apriltag
            (cX, cY) = (int(r.center[0]), int(r.center[1]))
            cv.circle(frame, (cX, cY), 5, (0, 0, 255), -1)

            # draw tag family on image
            tagFamily = r.tag_family
            tagID = r.tag_id
            cv.putText(frame, str(tagFamily)[2:-1] + " " + str(tagID), (ptD[0]-100,ptD[1]-20), 
                    cv.FONT_HERSHEY_COMPLEX, 2, (0, 255, 0), 4)

            # print(str(r.tag_family)[2:-1] + " ID = " + str(r.tag_id) + ", with decision margin " + str(r.decision_margin))
            # print("Pose estimation:")
            # print("rotation:")
            # print(r.pose_R)
            # print("translation:")
            # print(r.pose_t)
            # print("distance:")

            distance[count] = np.linalg.norm(r.pose_t)
            count += 1

            cv.putText(frame, str(distance), (ptD[0]-200,ptD[1]-120), cv.FONT_HERSHEY_COMPLEX, 2, (0, 255, 0), 4)
    
        return frame

