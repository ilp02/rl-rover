import cv2
import numpy as np
import time
import threading
import socket
import struct
from KalmanHelper import CVKalman2D

lock = threading.Lock()
# nn models takes:
# posx, posz, vx, vz, relx, relz, relvx, relvz, yaw
# aruco gives in meters
#  O---------  +x (origin is center of quad)
#  |
#  |
#  +z



addr_a0 = ("192.168.0.52", 9991)
addr_a1 = ("192.168.0.51", 9991)

# server-side datas
markers = [[0, 0, 0, 0], [0, 0, 0, 0]] # [a0pos, a1pos], a0pos = [x, y, z, yaw]
a0_v = np.zeros(2)
a1_v = np.zeros(2)

# robot-side datas
a_data = [[], []] #[accX, accY, yaw]

# flags
terminate = False
started = False

DEBUG = True

# TODO:
# Software
#   add better velocity
#   do calibrations better!!!! <5cm errors;;

def recvall(sock: socket.socket, n: int) -> bytes:
    """Read exactly *n* bytes or raise ConnectionError."""
    buf = b''
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:                       # peer closed
            raise ConnectionError("socket closed")
        buf += chunk
    return buf

pos_offset = np.array([-0.75, -0.5])
def talk_a0(addr):
    global a_data
    with socket.create_connection(addr) as sock:
        while not terminate:
            # send: vx, vz, px, pz, yaw, rpx, rpz, rvx, rvz (all doubles)
            with lock:
                sock.sendall(struct.pack('!9d', *a0_v*10, 
                                         *(markers[0][:2]+pos_offset)*10, 
                                         ((markers[0][3]+90)%360 +180)%360-180, 
                                         *(np.array(markers[0]) - np.array(markers[1]))[:2]*10, 
                                         *(a0_v - a1_v)*10))
                            
            # receive: accx, accy, yaw (all doubles)
            data = recvall(sock, 24)
            
            if not data:
                continue
            
            with lock:
                a_data[0] = list(struct.unpack('!3d', data))

def talk_a1(addr):
    global a_data
    with socket.create_connection(addr) as sock:
        while not terminate:
            # send: vx, vz, px, pz, yaw, rpx, rpz, rvx, rvz (all doubles)
            with lock:
                sock.sendall(struct.pack('!9d', *a1_v*10, 
                                         *(markers[1][:2]+pos_offset)*10, 
                                         ((markers[1][3]+90)%360 +180)%360-180, 
                                         *(np.array(markers[1]) - np.array(markers[0]))[:2]*10, 
                                         *(a1_v - a0_v)*10))
                                            
            # receive: accx, accy, yaw (all doubles)
            data = recvall(sock, 24)
            
            if not data:
                continue
            
            with lock:
                a_data[1] = list(struct.unpack('!3d', data))

def detect_markers():
    # id 0 - 3 is for corners. id 0 is the origin of all coordinates.
    # on the pi, there is two threads: one for comms, and another for nn model.
    # the socket thread uses async to communicate on a single thread, so the server should be aware.
    # the nn model would prob be resource intensive, so maybe multiprocessing would fit better..? idk

    # on the server there is three threads for game:
    # a-0 comms, a-1 comms, and main.
    # for server each comms a) sends pos and angle, b) recieves acc and gyro data, and c) terminates game if nessecery.
    # and the main thread reads all those data from a global var, check for termination, and sends termination signal.
    # the main thread reads a-0, a-1, aruco threads and sends data for each of them

    # before all threads have been developed, lets first write the part for processing aruco thread's data.
    global terminate
    global a0_v
    global a1_v
    global markers
    global started

    kf1 = CVKalman2D(sigma_pos=0.025, sigma_acc=2.0, gate_sigma=3.0)
    kf2 = CVKalman2D(sigma_pos=0.025, sigma_acc=2.0, gate_sigma=3.0)
    
    # 캘리브레이션 데이터 추출
    camera_matrix = np.load(r"D:\anaconda3\envs\py312\codes\newWeekNewME\cali_results\camera_matrix.npy")
    dist_coeffs = np.load(r"D:\anaconda3\envs\py312\codes\newWeekNewME\cali_results\dist_coeffs.npy")

    # ArUco 검출기 설정
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
    aruco_params = cv2.aruco.DetectorParameters()
    aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX   # or _CONTOUR / _APRILTAG
    aruco_params.cornerRefinementWinSize = 7        # pixels (odd number, try 5-11)
    aruco_params.cornerRefinementMaxIterations = 50
    aruco_params.cornerRefinementMinAccuracy = 0.01
    detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
    
    # 마커 크기 설정 (미터 단위)
    marker_size = 0.05  # 예: 5cm = 0.05m

    FPS = 30.0                                     # your camera rate
    DT  = 1.0 / FPS                                # 0.033 s
    WIN = 7                                        # SavGol window length (odd ≥ 5)
    POLY = 2                                       # quadratic fit

    # 카메라 설정
    cap = cv2.VideoCapture(1)

    # 카메라 초기화 대기
    with lock:
        started = True
    
    try:
        a0_old = []
        a1_old = []
        t_old = time.time()

        tvec_table = np.zeros((6, 1, 3))
        rvec_table = np.zeros((6, 1, 3))
        while not terminate:
            markers_buffer = []
            ret, frame = cap.read()
            if not ret:
                print("Failed to grab frame")
                break
                
            # 이미지 왜곡 보정
            frame_undistorted = cv2.undistort(frame, camera_matrix, dist_coeffs)
            
            # 마커 검출
            corners, ids, rejected = detector.detectMarkers(frame_undistorted)
            
            # 마커가 검출되면 표시 및 포즈 추정
            if ids is not None:
                # 검출된 마커 표시
                cv2.aruco.drawDetectedMarkers(frame_undistorted, corners, ids)
                
                # 각 마커의 포즈 추정
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners, marker_size, camera_matrix, dist_coeffs
                )

                for i, marker_id in enumerate(ids.flatten()):
                    if marker_id <= 5:
                        tvec_table[marker_id, 0] = tvecs[i, 0]
                        rvec_table[marker_id, 0] = rvecs[i, 0]
                
                # 각 마커에 대해 처리
                if DEBUG:
                    for i in range(len(ids)):
                        # 좌표축 표시
                        cv2.drawFrameAxes(frame_undistorted, camera_matrix, dist_coeffs, 
                                        rvecs[i], tvecs[i], marker_size/2)
                        
                        # 회전 벡터를 오일러 각도로 변환
                        rot_matrix, _ = cv2.Rodrigues(rvecs[i])
                        euler_angles = cv2.RQDecomp3x3(rot_matrix)[0]

                        # 마커의 3D 위치 표시
                        pos_x = tvecs[i][0][0]
                        pos_y = tvecs[i][0][1]
                        pos_z = tvecs[i][0][2]

                        # 마커 정보 표시
                        corner = corners[i][0]
                        center_x = int(np.mean(corner[:, 0]))
                        center_y = int(np.mean(corner[:, 1]))
                        
                        cv2.putText(frame_undistorted, 
                                f"ID: {ids[i][0]}", 
                                (center_x, center_y - 40), 
                                cv2.FONT_HERSHEY_SIMPLEX, 
                                0.5, (0, 0, 0), 2)
                                
                        cv2.putText(frame_undistorted,
                                f"Pos: ({pos_x:.2f}, {pos_y:.2f}, {pos_z:.2f})m",
                                (center_x, center_y),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.5, (0, 0, 0), 2)
                                
                        cv2.putText(frame_undistorted,
                                f"Rot: ({euler_angles[0]:.1f}, {euler_angles[1]:.1f}, {euler_angles[2]:.1f})deg",
                                (center_x, center_y + 20),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.5, (0, 0, 0), 2)
                        
                        # 코너 포인트 표시
                        for point in corner:
                            x, y = int(point[0]), int(point[1])
                            cv2.circle(frame_undistorted, (x, y), 4, (0, 0, 255), -1)
                    
                    cv2.imshow('ArUco Marker Detection', frame_undistorted)
                    
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        pass
                    
                    print(np.linalg.norm(tvec_table[4][:2]-tvec_table[5][:2])) # distance between id0 and id1 in meters

                for i in range(6):
                    # 회전 벡터를 오일러 각도로 변환
                    rot_matrix, _ = cv2.Rodrigues(rvec_table[i])
                    euler_angles = cv2.RQDecomp3x3(rot_matrix)[0]

                    # in m
                    markers_buffer.append(np.array([*(tvec_table[i][0]), euler_angles[2]])) # position in 2d plane and the yaw
                
            
            
            # shit
            m = np.array(markers_buffer)
            zplane = np.mean(m[:, 2])
            m -= np.array([m[0][0], m[0][1], zplane, m[0][3]]) # put them all on the id=0 marker's respective coords

            a0 = m[4]
            a1 = m[5]

            if len(a0_old)==0:
                a0_old = a0
                a1_old = a1
            
            # filter vel
            with lock:
                markers = [a0, a1]
    
                dt = time.time()-t_old
                kf1.predict(dt)
                kf2.predict(dt)
                if np.sum(a0) !=0 and np.sum(a1) != 0:
                    kf1.update(a0[:2])
                    kf2.update(a1[:2])
                    a0_v = kf1.vel
                    a1_v = kf2.vel
                    t_old = time.time()
                #print("A1 vel: ", a0_v)
                #print("A2 vel: ", a1_v)

            # check if inside a quad-point zone
            def isLeft(A, B, P):
                # checks if point P is on the left side of AB.
                return ((B[0] - A[0])*(P[1] - A[1]) - (B[1] - A[1])*(P[0] - A[0])) < 0
            
            def isInside(P):
                allsides = 0
                for i in range(4):
                    allsides += isLeft(m[i], m[(i+1)%4], P)
                if allsides==0 or allsides==4: # check if the points are all in the same sides. (all 0 or all 1)
                    return True
                else:
                    return False
            
            if (isInside(a0) and isInside(a1)) and (np.linalg.norm(a0[:2] - a1[:2]) >= 0.16): # inside zone, not touching
                pass
            else:
                with lock:
                    terminate = True
                    print("fuck")

            # yay, all agents are alive!

            #------------------
            # end of while loop

        cap.release()
    finally:
        cap.release()

# fire the threads!
threads = []
threads.append(threading.Thread(target=detect_markers, daemon=True))
threads[0].start()

print("Wait for aruco to initialize...")
while not started:
    time.sleep(0.1)
print("starting")

threads.append(threading.Thread(target=talk_a0, args=(addr_a0,), daemon=True))
threads.append(threading.Thread(target=talk_a1, args=(addr_a1,), daemon=True))

for t in threads[1:]:
    t.start()

while not terminate:
    a = input("Terminate? [Y/[n]]")
    if a == "Y":
        with lock:
            terminate = True
        break

for t in threads:
    t.join()

print("Yippie!")
