import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2
from pupil_apriltags import Detector
from april_detect_msgs.msg import Detections, Tag
from rcl_interfaces.msg import ParameterDescriptor
import numpy as np
import time
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import message_filters


class AprilDetectSubscriber(Node):

    def __init__(self):
        super().__init__('april_detect_node')

        param_descriptor = ParameterDescriptor(description='fx Camera Intrisic.')
        self.declare_parameter('camera_intrinsics.fx', 0.0, param_descriptor)
        param_descriptor = ParameterDescriptor(description='fy Camera Intrisic.')
        self.declare_parameter('camera_intrinsics.fy', 0.0, param_descriptor)  
        param_descriptor = ParameterDescriptor(description='cx Camera Intrisic.')
        self.declare_parameter('camera_intrinsics.cx', 0.0, param_descriptor)
        param_descriptor = ParameterDescriptor(description='cy Camera Intrisic.')
        self.declare_parameter('camera_intrinsics.cy', 0.0, param_descriptor)  
        param_descriptor = ParameterDescriptor(description='Stereo Baseline.')
        self.declare_parameter('camera_intrinsics.b', 0.0, param_descriptor)  
        param_descriptor = ParameterDescriptor(description='Stereo Img Topic.')
        self.declare_parameter('stereo_img_topic', "", param_descriptor)  
        param_descriptor = ParameterDescriptor(description='Stereo Frame ID.')
        self.declare_parameter('stereo_frame_id', "", param_descriptor)  
        param_descriptor = ParameterDescriptor(description='Topic to Publish Detections [april_detect_msgs/msg/Detections]')
        self.declare_parameter('april_detections_topic', "", param_descriptor)
        param_descriptor = ParameterDescriptor(description='Total Number of Tags That can be Detected')
        self.declare_parameter('total_tags', 100, param_descriptor)
        param_descriptor = ParameterDescriptor(description='Time Window for Temporal Filter')
        self.declare_parameter('time_window', 100, param_descriptor)
        param_descriptor = ParameterDescriptor(description='STD of x coordinate via Triangulation')
        self.declare_parameter('triangulation_stds.x', 0.0, param_descriptor)
        param_descriptor = ParameterDescriptor(description='STD of y coordinate via Triangulation')
        self.declare_parameter('triangulation_stds.y', 0.0, param_descriptor)
        param_descriptor = ParameterDescriptor(description='STD of z coordinate via Triangulation')
        self.declare_parameter('triangulation_stds.z', 0.0, param_descriptor)
        param_descriptor = ParameterDescriptor(description='STD of x coordinate via Homography')
        self.declare_parameter('homography_stds.x', 0.0, param_descriptor)
        param_descriptor = ParameterDescriptor(description='STD of y coordinate via Homography')
        self.declare_parameter('homography_stds.y', 0.0, param_descriptor)
        param_descriptor = ParameterDescriptor(description='STD of z coordinate via Homography')
        self.declare_parameter('homography_stds.z', 0.0, param_descriptor)
        param_descriptor = ParameterDescriptor(description='Zoom Decimation')
        self.declare_parameter('quad_decimate', 0.0, param_descriptor)
        param_descriptor = ParameterDescriptor(description='Blurry Sigma')
        self.declare_parameter('quad_sigma', 0.0, param_descriptor)
        param_descriptor = ParameterDescriptor(description='Sharpening Coefficient')
        self.declare_parameter('decode_sharpening', 0.0, param_descriptor)
        param_descriptor = ParameterDescriptor(description='AprilTag Side Length')
        self.declare_parameter('tag_size', 0.0, param_descriptor)


        # Subscription To Stereo Image
        self.stereo_img_input_sub = self.create_subscription(CompressedImage, str(self.get_parameter('stereo_img_topic').value), self.detect_callback, 1)
        self.stereo_img_input_sub  # prevent unused variable warning

        # Publish Detections
        self.detections_publisher = self.create_publisher(Detections, str(self.get_parameter('april_detections_topic').value), 10)
        self.detections_msg = Detections()

        # Define April Tag Detector (Pupil - Labs)
        self.detector = Detector(
                                families="tag36h11",
                                nthreads=4,
                                quad_decimate=self.get_parameter('quad_decimate').value,
                                quad_sigma=self.get_parameter('quad_sigma').value,
                                refine_edges=1,
                                decode_sharpening=self.get_parameter('decode_sharpening').value,
                                debug=0
                                )

        self.tag_size = self.get_parameter('tag_size').value
        # CVBridge Object
        self.br = CvBridge()

        # tf broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        self.total_tags = int(self.get_parameter('total_tags').value)
        self.tag_present_left = np.zeros(self.total_tags).astype(bool)
        self.tag_present_right = np.zeros(self.total_tags).astype(bool)
        self.tag_centers_left = np.zeros((2, self.total_tags)).astype(int)
        self.tag_centers_right = np.zeros((2, self.total_tags)).astype(int)

        # Time Window for temporal Filtering
        self.time_window = int(self.get_parameter('time_window').value)

        print("time_window", self.time_window)
    
        # Time Filter
        self.tag_present_left_time_filter = np.zeros(self.total_tags).astype(int)
        self.tag_present_right_time_filter = np.zeros(self.total_tags).astype(int)

        # Storage for Homography Poses Computed from the Left Camera
        self.homography_poses_left = np.zeros((3, self.total_tags))

        # Storage for Homography Poses Computed from the Right Camera
        self.homography_poses_right = np.zeros((3, self.total_tags))

        # Rotation for Homography Calculated Poses to place them into the Correct Optical Frame
        self.hom_rot = np.asarray([[-1., 0., 0.], [0., -1., 0.], [0., 0., 1.]])

        # Array of Indices for Correct Marker ID Visualization
        self.indixes = np.array(range(self.total_tags))

        # simulation
        # Camera Intrinsic Parameters
        self.fx = float(self.get_parameter('camera_intrinsics.fx').value)
        self.fy = float(self.get_parameter('camera_intrinsics.fy').value)
        self.cx = float(self.get_parameter('camera_intrinsics.cx').value)
        self.cy = float(self.get_parameter('camera_intrinsics.cy').value)
        self.b =  float(self.get_parameter('camera_intrinsics.b').value)

        self.frame_id = str(self.get_parameter('stereo_frame_id').value)

        # STD Values for Q
        std_x_trian = float(self.get_parameter('triangulation_stds.x').value)
        std_y_trian = float(self.get_parameter('triangulation_stds.y').value) 
        std_z_trian = float(self.get_parameter('triangulation_stds.z').value)

        # STD Values for R
        std_x_hom = float(self.get_parameter('homography_stds.x').value)
        std_y_hom = float(self.get_parameter('homography_stds.y').value)
        std_z_hom = float(self.get_parameter('homography_stds.z').value)

        # Set Process and Measurement Noise
        self.Q = np.array([[std_x_trian**2, 0, 0], 
                           [0, std_y_trian**2, 0], 
                           [0, 0, std_z_trian**2]])

        self.R = np.array([[std_x_hom**2, 0, 0], 
                           [0, std_y_hom**2, 0], 
                           [0, 0, std_z_hom**2]])

        self.counter = 0


        imageL_sub = message_filters.Subscriber("/camera/left/image_raw", Image)
        imageR_sub = message_filters.Subscriber("/camera/right/image_raw", Image)

        # Synchronize images
        ts = message_filters.ApproximateTimeSynchronizer([imageL_sub, imageR_sub], queue_size=10, slop=0.5)
        ts.registerCallback(image_callback)

    def detect_callback(self, msg):

        # [x, y] = [col, row]
        st = time.time()

        # Use CVBridge to get sensor_msgs/msg/Image and convert it to OpenCV
        stereo_img = self.br.compressed_imgmsg_to_cv2(msg)

        # cv2.imwrite("/home/enrique/test_imgs/"+"img_"+str(self.counter)+".png", stereo_img)
        self.counter += 1

        print("img_received")

        # convert Received Image to GrayScale
        stereo_img_input = cv2.cvtColor(stereo_img, cv2.COLOR_BGR2GRAY)

        # Compute Width and Height of an Individual Image
        img_width = int(stereo_img_input.shape[1]/2)
        img_height = stereo_img_input.shape[0]

        # Detect April-Tags on Left Image
        img_left = stereo_img_input[:, 0:img_width]

        # Detect April-Tags on Right Image
        img_right = stereo_img_input[:, img_width:]

        # Detect tags 
        # Sim
        results_left = self.detector.detect(img_left, estimate_tag_pose = True, camera_params=[self.fx, self.fy, self.cx, self.cy], tag_size = self.tag_size)
        results_right = self.detector.detect(img_right, estimate_tag_pose = True, camera_params=[self.fx, self.fy, self.cx, self.cy], tag_size = self.tag_size)

        # Return from Function if not Tag is Detected
        if len(results_left) == 0 and len(results_right) == 0:
            return

        # For Each Detected AprilTag   
        for r in results_left:

            # Get Tag Center Coordinates
            center = np.asarray(r.center, dtype = int).reshape((2, 1))

            # Get Tag ID
            id = r.tag_id

            # Mark Tag as Present in the Current detection
            self.tag_present_left[id] = 1

            # Save the Tag Center
            self.tag_centers_left[:, [id]] = center 

            # Save Tag Pose Computed Via Homography in self.homography_poses_left from Left Img
            self.homography_poses_left[:, [id]] =  self.hom_rot@np.asarray(r.pose_t)   

            cv2.circle(stereo_img, center[:, 0], 5, (0, 0, 255), 5)
 
        # Retrieve Detections from 
        for r in results_right: 

            # Get Tag Center Coordinates
            center = np.asarray(r.center, dtype = int).reshape((2, 1))

            # Get Tag ID
            id = r.tag_id

            # Mark Tag as present in the Current detection
            self.tag_present_right[id] = 1

            # Save the Tag Center
            self.tag_centers_right[:, [id]] = center

            # Save Tag Pose Computed Via Homography in self.homography_poses_right from Right Img
            self.homography_poses_right[:, [id]] =  self.hom_rot@np.asarray(r.pose_t) 

            self.homography_poses_right[0, [id]] -= self.b

            cv2.circle(stereo_img, center[:, 0] + np.array([img_width, 0]), 5, (0, 0, 255), 5)

        # Check for Tag Detections Along Time
        self.tag_present_left_time_filter += self.tag_present_left
        self.tag_present_right_time_filter += self.tag_present_right

        # Take Tags that are Present During the Whole time Window
        self.tag_present_left = np.where(self.tag_present_left_time_filter == self.time_window, True, False)
        self.tag_present_right = np.where(self.tag_present_right_time_filter == self.time_window, True, False)

        # Show Computation Time Until This Point

        # Find tags Detected in Both Cameras
        associations = np.logical_and(self.tag_present_left, self.tag_present_right)

        # Return from Function When a Landmark is not seen by both cameras
        if np.sum(self.tag_present_left) > 0 and np.sum(self.tag_present_right) == 0:
            '''
            Publish 3D Tag Poses Computed from the Left Camera
            '''
            print("Just Left")

            homography_poses_left = self.homography_poses_left[:, self.tag_present_left]

            self.handle_pose(self.indixes[self.tag_present_left], homography_poses_left, msg.header.stamp)

        elif np.sum(self.tag_present_right) > 0 and np.sum(self.tag_present_left) == 0:
            '''
            Publish 3D Tag Poses Computed from the Right Camera
            '''
            print("Just Right")

            homography_poses_right = self.homography_poses_right[:, self.tag_present_right]

            self.handle_pose(self.indixes[self.tag_present_right], homography_poses_right, msg.header.stamp)

        elif np.sum(associations) == 0:
            '''
            If There is No Association in the Current Images 
            Return
            '''
            print("no association")
            return
        
        else:
            '''
            Compute 3D poses of Detected Landmarks by Both Cameras via Triangulation and 
            improve it via Fusion of the 3D poses computed from Homographies from Both 
            Cameras suing A Linar Kalman Filter
            '''
            print("Fusing 5")

            # Get Left Camera X Coordinates
            u1 = self.tag_centers_left[0, associations] 
            # Get Right Camera X Coordinates
            u2 = self.tag_centers_right[0, associations] 

            # Get Cameras' Y Coordinates
            v1 = v2 = self.tag_centers_left[1, associations] - self.cy
            
            # Compute Disparities
            d = u1 - u2

            # Compute Depth for each Disparity 
            Z = self.b*self.fx/d

            # Compute X Coordinate for each Disparity 
            X = -self.b*(u1 + u2 - 2*self.cx)/(2*d)

            # Compute Y Coordinate for each Disparity 
            Y = -self.b * v1/d
            
            # 3D poses Computed Via Stereo Triangulation
            triangulation_poses = np.vstack((X, Y, Z))

            # 3D Poses Computed Homographies Via Known Tag Dimensions from Left Camera
            homography_poses_left = self.homography_poses_left[:, associations]

            # 3D Poses Computed Homographies Via Known Tag Dimensions from Right Camera
            homography_poses_right = self.homography_poses_right[:, associations]

            # Improve Triangulated Pose Via poses obtained from Left Homographies
            poses_opt, P_opt = self.fuse(triangulation_poses, homography_poses_left, self.Q, self.R)
            
            # Improve Triangulated Pose Via poses obtained from Right Homographies
            poses_opt, _ = self.fuse(poses_opt, homography_poses_right, self.Q, self.R, P = P_opt)

            # poses_opt = (homography_poses_right + homography_poses_left)/2
            poses_opt = homography_poses_right

            # Broadcast Transform and Publish Message
            self.handle_pose(self.indixes[associations], poses_opt, msg.header.stamp)


            cv2.imshow("camera", stereo_img)
            
            cv2.waitKey(1)

        et = time.time()
        elapsed_time = et - st
        print("elapsed_time: ", elapsed_time)

        # Clear Tag Presence Containers
        self.tag_present_left = np.zeros(self.total_tags).astype(bool)
        self.tag_present_right = np.zeros(self.total_tags).astype(bool)

        # Clear Time Filters Containers
        self.tag_present_left_time_filter = np.zeros(self.total_tags)
        self.tag_present_right_time_filter = np.zeros(self.total_tags)


    def handle_pose(self, ids, poses, stamp):

        # create Transoform Stamped Msg
        tfs = TransformStamped()

        # crete an Array Containing the Detections
        detection_array = []

        # Publish and Broadcast Every Detection
        for i in range(poses.shape[1]):

            # Create a New Tag msg
            tag_msg = Tag()

            # Set the ID of the detected Landmark
            tag_msg.id = int(ids[i])

            print("HOHOHOHOHOH")

            # Set Header for a Given TF
            tfs.header.stamp = self.get_clock().now().to_msg()
            tfs.header.frame_id = self.frame_id
            tfs._child_frame_id = str(ids[i]) + "_detection"

            # Set X, Y, Z Measured Coordinates of a Given Detection
            tag_msg.x = tfs.transform.translation.x = poses[0, i]
            tag_msg.y = tfs.transform.translation.y = poses[1, i]
            tag_msg.z = tfs.transform.translation.z = poses[2, i]

            # Set Orientation of a Given Detection
            tfs.transform.rotation.x = 0.
            tfs.transform.rotation.y = 0.
            tfs.transform.rotation.z = 0.
            tfs.transform.rotation.w = 1.

            # Broadcast Individual Detecction TF
            self.tf_broadcaster.sendTransform(tfs)
            
            # Append A given Detection to the detection_array
            detection_array.append(tag_msg)

        # Assign the list of detections to the detections_msg
        self.detections_msg.detections = detection_array

        # Publish the Detections
        self.detections_publisher.publish(self.detections_msg)


    def fuse(self, X, Z, q, r, P = None): 
        ''' 
        Q is the Individual Process Noise
        R is the Individual Measurement Noise
        '''
        # Get Dimensions For Stacking Multiple Measurements
        measurement_dim = X.shape[0]
        n_instances = X.shape[1]
        total_size = n_instances*measurement_dim

        # Initialize P if not Defined in the Function Call
        if P is None:
            P = np.identity(total_size) * 0.5 

        # Initialize and Populate KF Matrices
        Q = np.zeros((total_size, total_size))

        R = np.zeros((total_size, total_size))

        F = np.identity(total_size)

        H = np.identity(total_size)

        I = np.identity(total_size)

        X_1 = X.reshape((total_size, 1))

        Z_1 = Z.reshape((total_size, 1))

        # Stack Covariance Matrices
        for i in range(n_instances):
            Q[i*measurement_dim:i*measurement_dim + measurement_dim, i*measurement_dim:i*measurement_dim + measurement_dim] = q
            R[i*measurement_dim:i*measurement_dim + measurement_dim, i*measurement_dim:i*measurement_dim + measurement_dim] = r

        # Compute Prediction Covariance P
        P_1 =  F@P@F.T + Q

        # Compute Kalman Gain
        K = P_1@H.T @ np.linalg.inv(H@P_1@H.T + R)

        # Compute Updated Mean
        X_fused = X_1 + K@(Z_1 - H@X_1)

        # Compute Updated Covariance
        P_fused = (I - K@H)@P_1@(I - K@H).T + K@R@K.T

        return X_fused.reshape((measurement_dim, n_instances)), P_fused

def main(args=None):
    print("Starting April Robust Triangulation Node From Stereo Images")

    rclpy.init(args=args)
    stereo_glue_sub = AprilDetectSubscriber()
    rclpy.spin(stereo_glue_sub)

    print("Destroying Stereo Glue Node")
    stereo_glue_sub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()