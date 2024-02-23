from pupil_apriltags import Detector
import cv2
import numpy as np
import time
from robomaster import robot
from robomaster import camera

at_detector = Detector(
    families="tag36h11",
    nthreads=1,
    quad_decimate=1.0,
    quad_sigma=0.0,
    refine_edges=1,
    decode_sharpening=0.25,
    debug=0
)

def find_pose_from_tag(K, detection):
    m_half_size = tag_size / 2

    marker_center = np.array((0, 0, 0))
    marker_points = []
    marker_points.append(marker_center + (-m_half_size, m_half_size, 0))
    marker_points.append(marker_center + ( m_half_size, m_half_size, 0))
    marker_points.append(marker_center + ( m_half_size, -m_half_size, 0))
    marker_points.append(marker_center + (-m_half_size, -m_half_size, 0))
    _marker_points = np.array(marker_points)

    object_points = _marker_points
    image_points = detection.corners

    pnp_ret = cv2.solvePnP(object_points, image_points, K, distCoeffs=None,flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if pnp_ret[0] == False:
        raise Exception('Error solving PnP')

    r = pnp_ret[1]
    p = pnp_ret[2]

    return p.reshape((3,)), r.reshape((3,))


if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")

    ep_chassis = ep_robot.chassis

    ep_camera = ep_robot.camera
    ep_camera.start_video_stream(display=False, resolution=camera.STREAM_360P)

    tag_size=0.16 # tag size in meters  -we used an ipad size is different 

    while True:
        try:
            img = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)  
            cv2.imwrite("/home/user/Desktop/test.png", img) 
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            gray.astype(np.uint8)

            K=np.array([[184.752, 0, 320], [0, 184.752, 180], [0, 0, 1]])

            results = at_detector.detect(gray, estimate_tag_pose=False)
            for res in results:
                #print(res)
                pose = find_pose_from_tag(K, res)
                rot, jaco = cv2.Rodrigues(pose[1], pose[1])

                scalar_x = 2*(pose[0][2]-.25) #equals distance from ideal spot from marker (.25 m away)
                scalar_y = 2.5*pose[0][0] #controls displacement left-to-right from marker
                scalar_z = 3*((pose[1][2]))*180 / np.pi
                print(scalar_z)
                """
                if scalar_x < 0 :
                    ep_chassis.drive_speed(x=-(-.35 + scalar_x*3)**2, y=scalar_y, z=3.5*scalar_z, timeout=5)
                else :
                    ep_chassis.drive_speed(x=2*scalar_x**2, y=scalar_y, z=1.5*scalar_z, timeout=5)
                ## for next time - change how scalar_z is +5 or -5 based on its sign
                """
                """
                             if scalar_x < 0 and scalar_z <1 :
                    ep_chassis.drive_speed(x=-(-.35 + scalar_x*3)**2, y=scalar_y, z=5*(5+scalar_z), timeout=5)
                elif scalar_x < 0 and scalar_z >1 :
                    ep_chassis.drive_speed(x=-(-.35 + scalar_x*3)**2, y=scalar_y, z=1.5*scalar_z, timeout=5)
                else :
                    ep_chassis.drive_speed(x=2*scalar_x**2, y=scalar_y, z=1.5*scalar_z, timeout=5)
                """
                                
                
                #print("rotation matrix: ", rot)
                #print("Post 1: ", pose[1])


                pts = res.corners.reshape((-1, 1, 2)).astype(np.int32)
                img = cv2.polylines(img, [pts], isClosed=True, color=(0, 0, 255), thickness=5)
                cv2.circle(img, tuple(res.center.astype(np.int32)), 5, (0, 0, 255), -1)
                #print("Distance is: ", pose[0][2])
                #print("Displacement is: ", pose[0][0])
                

            cv2.imshow("img", img)
            cv2.waitKey(10)

        except KeyboardInterrupt:
            ep_camera.stop_video_stream()
            ep_robot.close()
            print ('Exiting')
            exit(1)


