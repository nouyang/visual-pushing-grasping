# http://www.philipzucker.com/aruco-in-opencv/
'''
Run as sanity check that we can open the webcam and detect a tag.

Date: 18 Jan 2019
Author: nouyang

(Passing note: apparently cv2.aruco detection use the apriltag algorithm?
https://github.com/opencv/opencv_contrib/pull/1637)

'''

import cv2
import cv2.aruco as aruco
import numpy as np
import math
import matplotlib.pyplot as plt
import sys
import time
import pprint
import copy
import os

import urx

tcp_host_ip = "10.75.15.91"
# rob = urx.Robot(tcp_host_ip)

# rob.set_tcp((0, 0, 0, 0, 0, 0))
# rob.set_payload(0.5, (0, 0, 0))


# delta = 0.05
# delta = 0.001
# v = 0.05
# a = 0.3
# pose = rob.getl()
# print("robot tcp is at: ", pose, '\n')


# def moveTool(flagMove):
# if flagMove:
# rob.translate_tool((0, 0, delta), acc=a, vel=v)


with open('sensor_log.txt', 'w') as outf:
    outf.flush()

# ---------- CHANGE DEVICE NUMBER AS NEEDED
if len(sys.argv) > 1:
    CAM_NUM = int(sys.argv[1])
    print('trying camera', CAM_NUM)
    cap = cv2.VideoCapture(CAM_NUM)
else:
    cap = cv2.VideoCapture(0)
# help(cv2.aruco)

# aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
# https://github.com/opencv/opencv_contrib/blob/96ea9a0d8a2dee4ec97ebdec8f79f3c8a24de3b0/modules/aruco/samples/create_board.cpp
# "{d        |       | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, DICT_4X4_250=2,"
# "DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, DICT_5X5_250=6, DICT_5X5_1000=7, "
# "DICT_6X6_50=8, DICT_6X6_100=9, DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12,"
# "DICT_7X7_100=13, DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16}"

print(aruco_dict)

'''    detectMarkers(...)
        detectMarkers(image, dictionary[, corners[, ids[, parameters[, rejectedI
        mgPoints]]]]) -> corners, ids, rejectedImgPoints
'''


# strtime = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
# apriltime = time.time()

# --------------------
# NOTE: Not sure where these came from. (maybe the HD webcam?) Camera matrix constants

# cameraMatrix = np.array([[619.6207333698724, 0.0, 283.87165814439834],
# [0.0, 613.2842389650563, 231.9993874728696],
# [0.0, 0.0, 1.0]])
# distCoeffs, rvecs, tvecs = np.array([]), [], []

np.set_printoptions(suppress=True, precision=1)


# --------------------
# Camera matrix constants

# NOTE: I do not have actual camera matrix for the Amazon hawk sensor! (original
# flat)
# Approximate using another webcam...  HBV-1716-2019-01-29_calib.yaml

cameraMatrix = np.array([[521.92676671,   0.,         315.15287785],
                         [0.,         519.01808261, 193.05763006],
                         [0.,           0.,           1.]])

distCoeffs, rvecs, tvecs = np.array([]), [], []

distCoeffs = np.array([0.02207713,  0.18641578, -0.01917194, -0.01310851,
                       -0.11910311])


# ------------------------------
distBtwTags = 0.004  # center to center
tagSize = 0.0038
translate_vec = np.array([45, 0, 0])
euler_xyz_vec = np.array([45, 0, 0])  # in degrees

# Checks if a matrix is a valid rotation matrix.


def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6


# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R):

    assert(isRotationMatrix(R))

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6
    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])  # roll
        y = math.atan2(-R[2, 0], sy)  # pitch
        z = math.atan2(R[1, 0], R[0, 0])  # yaw
    else:  # gimbal lock
        print('gimbal lock')
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    # TODO: do I need this?

    rots = np.array([x, y, z])
    rots = np.array([math.degrees(r) for r in rots])

    # rots[0] = 360 - rots[0] % 360 # change rotation sense if needed, comment this line otherwise
    rots[0] = 180 - rots[0] % 360
    # rots[1]  = rots[1] + 90

    # print(math.atan(R[2,0] / R[0,0]) / math.pi * 180,
    # math.atan(R[2,1] / R[1,1]) / math.pi * 180,
    # math.atan(R[1,0] / R[0,0]) / math.pi * 180)

    # print('with negs', rots)
    # for (i, r) in enumerate(rots):
    # if r < -90:
    # rots[i] = 360 + r
    return rots

# ------------------------------


# TODO --------
# out, prevOut = np.array([180,10,5]), np.array([180,10,5])
numTags = 2
out, prevOut = np.ones((numTags, 3)), np.ones((numTags, 3))
# w = 0.2 # 0.2 = fairly heavy weighting
WEIGHT = 0.2
# plt.axis([0,10,0,1])
count = 0
x, y = [0], [-45]
fig = plt.figure()

starttime, inittime = time.time(), time.time()
now = 0
elapsed = 0
rate = 0
plt.plot(count, y)
plt.title('Example Fiducial Force-Torque Sensor Interface')
plt.xlabel('Time (s)')
plt.ylabel('Y Readings (not real units)')

AXIZ = 0

rvec, tvec = None, None
fields = ['Fx: ', 'Fy: ', 'Fz: ', 'Mx: ', 'My: ', 'Mz: ']

readings = []
x_readings = []
y_readings = []


# ---------------------------------------------------------
# Start program
# ---------------------------------------------------------

# -----------------------------------
# Initialized zeros
counter = 0
avgN = 20
cumTvec, cumRvec = np.ones((numTags, 3)), np.ones((numTags, 3))

# -- NOTE: this code is highly repetitive, refactor at some point
while counter < avgN:
    ret, frame = cap.read(0.)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
    aruco_params = aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = \
        aruco.detectMarkers(gray,
                            aruco_dict,
                            parameters=aruco_params)

    if (ids is not None) and (len(ids) == numTags):
        tagData = zip(ids, corners)
        # put corners in order of id
        tagData = sorted(tagData, key=lambda x: x[0])
        ids = [tag[0] for tag in tagData]
        corners = [tag[1] for tag in tagData]
    # print(ids)

        # rvec, tvec = \
    rvec, tvec, objPoints = \
        cv2.aruco.estimatePoseSingleMarkers(corners, tagSize,
                                            cameraMatrix,
                                            distCoeffs)

    if rvec is not None and rvec.shape[0] == numTags:
        counter += 1
        tvec = tvec.reshape(numTags, 3)

        cumTvec += tvec
        for i in range(numTags):
            rotMat, jacob = cv2.Rodrigues(rvec[i])
            rots = rotationMatrixToEulerAngles(rotMat)
            cumRvec[i] = rots


zeroThetas = copy.deepcopy(cumRvec/avgN)
zeroDists = copy.deepcopy(cumTvec/avgN)


out, prevOut = copy.deepcopy(zeroThetas), copy.deepcopy(zeroDists)

while(True):

    # ---------------------------------------------------------
    # Start program
    # ---------------------------------------------------------

    # Capture frame-by-frame
    ret, frame = cap.read(0.)
    # print(frame.shape) #480x640

    # Our operations on the frame come here
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
    # Param defaults:
    # https://docs.opencv.org/3.4/d1/dcd/structcv_1_1aruco_1_1DetectorParameters.html
    parameters = aruco.DetectorParameters_create()

    # lists of ids and the corners beloning to each id
    corners, ids, rejectedImgPoints = aruco.detectMarkers(
        gray, aruco_dict, parameters=parameters)
    # print(corners)

    gray = aruco.drawDetectedMarkers(gray, corners)
    cv2.imshow('frame', gray)

    if (ids is not None) and (len(ids) == numTags):
        tagData = zip(ids, corners)
        # put corners in order of id
        tagData = sorted(tagData, key=lambda x: x[0])
        ids = [tag[0] for tag in tagData]
        corners = [tag[1] for tag in tagData]

        # rvec, tvec = \
        rvec, tvec, objPoints = \
            cv2.aruco.estimatePoseSingleMarkers(corners, tagSize,
                                                cameraMatrix,
                                                distCoeffs)

    # convert rotMat to Euler, and smooth with filter
    if rvec is not None and rvec.shape[0] == numTags:
        count += 1
        if (count % 2) == 0:
            now = time.time()
            elapsed = now - starttime
            starttime = now
            rate = 2 / elapsed
        for i in range(numTags):
            rotMat, jacob = cv2.Rodrigues(rvec[i].flatten())
            rots = rotationMatrixToEulerAngles(rotMat)

            out[i] = WEIGHT * rots + (1 - WEIGHT) * prevOut[i]
            prevOut[i] = out[i]

        # NOTE: we are exponential filtering thetas but not distances
        calcDists = tvec.reshape((2, 3)) - zeroDists
        calcThetas = rvec.reshape((2, 3)) - zeroDists

        avgCalcDists = np.average(calcDists, axis=0)
        avgCalcThetas = np.average(calcThetas, axis=0)

        # print('tvec', tvec)
        # print('tvec shape', tvec.shape)
        # print('calcdist', calcDists)
        # print('calcdist shape', calcDists.shape)
        # print('zerodist', zeroDists)
        # print('zerodist shape', zeroDists.shape)
        # print('rvec', rvec)
        # print('rvec shape', rvec.shape)
        # print('calctheta', calcThetas)
        # print('calctheta shape', calcThetas.shape)
        # print('zerodist', zeroThetas)
        # print('zerodist shape', zeroThetas.shape)

        # Average between the two tags
        avgCalcDists = np.average(calcDists, axis=0)
        avgCalcThetas = np.average(calcThetas, axis=0)

        # print(rvec.shape)
        out = out.reshape((numTags, 1, 3))

        tagIdx = 1
        reading = np.concatenate(
            (tvec[tagIdx]*1000, rvec[tagIdx]*100)).flatten()
        reading = np.round(reading, decimals=3)
        readings.append(reading)
        # a = ['{0: <10}'.format(x) for x in reading]
        # pprint.pprint(''.join([val for pair in zip(fields, a) for val in
        # pair]))
        print('%.2f\n' % (calcDists[tagIdx][AXIZ]*10000))
        # print(count)
        # print(out.flatten()[AXIZ])
        #plt.plot(count, out.flatten()[AXIZ], 'k.')
        x_readings.append(count)
        y_readings.append(calcDists[tagIdx][AXIZ]*1000)
        # plt.gca().lines[0].set_xdata(x_readings)
        # plt.gca().lines[0].set_ydata(y_readings)
        # plt.gca().relim()
        # plt.gca().autoscale_view()
        # plt.pause(0.001)

        with open('sensor_log.txt', 'w') as outf:
            outf.write(str(calcDists[tagIdx][AXIZ]*1000) + '\n')
            outf.flush()
        time.sleep(0.01)

    # print(rejectedImgPoints)
    # Display the resulting frame
    if cv2.waitKey(1) & 0xFF == ord('s'):
        print(np.array(readings).std(axis=0)*1000)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        endtime = time.time()
        break

ls

# When everything done, release the capture
print('Program init time', inittime, 'endtime', endtime, 'counts', count)
print('Counts/sec (Hz)', count/(endtime - inittime))
cap.release()
cv2.destroyAllWindows()


# TypeError: Expected cv::UMat for argument
# rvec, tvec, objPoints = cv2.aruco.estimatePoseSingleMarkers(corners, 0.05, cameraMatrix, distCoeffs);
# labels=[0]*len(faces)
# face_recognizer.train(faces, np.array(labels))

# https://stackoverflow.com/questions/12933284/rodrigues-into-eulerangles-and-vice-versa
