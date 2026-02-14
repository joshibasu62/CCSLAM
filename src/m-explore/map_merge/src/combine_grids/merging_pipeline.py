import numpy as np
import cv2
import math
from geometry_msgs.msg import Transform
from grid_warper import GridWarper
from grid_compositor import GridCompositor

class MergingPipeline:
    def __init__(self):
        self.grids = []  # Original ROS messages
        self.images = [] # Converted numpy images
        self.transforms = [] # 3x3 matrices
        self.warper = GridWarper()
        self.compositor = GridCompositor()

    def feed(self, grids):
        self.grids = grids
        self.images = []
        for grid in grids:
            # Convert ROS OccupancyGrid to numpy array
            w = grid.info.width
            h = grid.info.height
            data = np.array(grid.data, dtype=np.int8).reshape((h, w))
            self.images.append(data)

    def set_transforms(self, transforms):
        """
        Sets known transforms (from params).
        Expects list of geometry_msgs/Transform
        """
        self.transforms = []
        for t in transforms:
            mat = self._ros_transform_to_matrix(t)
            self.transforms.append(mat)

    def estimate_transforms(self, feature_type='ORB', confidence=1.0):
        """
        Uses Feature matching to find transforms between maps.
        Simple implementation using ORB + RANSAC.
        """
        if not self.images:
            return False

        # If only one image, identity transform
        if len(self.images) == 1:
            self.transforms = [np.eye(3)]
            return True

        # Feature Detector
        # Note: AKAZE/SURF usage depends on OpenCV build. ORB is safe standard.
        if feature_type == 'AKAZE':
            detector = cv2.AKAZE_create()
        else:
            detector = cv2.ORB_create(nfeatures=2000)
            
        keypoints_list = []
        descriptors_list = []

        # Detect features
        for img in self.images:
            # Mask out unknown areas (-1) for detection
            mask = (img != -1).astype(np.uint8)
            # Convert to uint8 for OpenCV (0-255)
            # Map: -1->127 (Gray), 0->255 (White), 100->0 (Black) for visual logic
            # But for features, just normalizing to 0-255 is usually enough
            vis_img = np.zeros_like(img, dtype=np.uint8)
            vis_img[img == 0] = 255   # Free space
            vis_img[img == 100] = 0   # Walls
            
            kp, des = detector.detectAndCompute(vis_img, mask)
            keypoints_list.append(kp)
            descriptors_list.append(des)

        # Matcher
        bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        
        self.transforms = [np.eye(3)] * len(self.images)
        
        # Pairwise matching against index 0 (Reference Frame)
        # This is a simplified logic compared to full bundle adjustment
        ref_kp = keypoints_list[0]
        ref_des = descriptors_list[0]

        if ref_des is None:
            return False

        for i in range(1, len(self.images)):
            curr_kp = keypoints_list[i]
            curr_des = descriptors_list[i]
            
            if curr_des is None:
                continue

            matches = bf.match(curr_des, ref_des)
            matches = sorted(matches, key=lambda x: x.distance)

            if len(matches) < 10:
                continue

            # Extract points
            src_pts = np.float32([curr_kp[m.queryIdx].pt for m in matches]).reshape(-1, 1, 2)
            dst_pts = np.float32([ref_kp[m.trainIdx].pt for m in matches]).reshape(-1, 1, 2)

            # Estimate affine transform
            # estimateAffinePartial2D handles rotation + translation + uniform scale
            M, mask = cv2.estimateAffinePartial2D(src_pts, dst_pts)
            
            if M is not None:
                # Convert 2x3 to 3x3
                M_3x3 = np.eye(3)
                M_3x3[0:2, :] = M
                self.transforms[i] = M_3x3

        return True

    def compose_grids(self):
        if not self.images or len(self.images) != len(self.transforms):
            return None

        warped_grids = []
        rois = []

        # Warp images
        for i, img in enumerate(self.images):
            warped, roi = self.warper.warp(img, self.transforms[i])
            warped_grids.append(warped)
            rois.append(roi)

        # Compose
        result_msg, min_x, min_y = self.compositor.compose(warped_grids, rois)
        if result_msg is None:
            return None

        # Set Metadata
        # Use resolution of the first grid (reference)
        resolution = self.grids[0].info.resolution
        result_msg.info.resolution = resolution
        
        # Calculate origin
        # The result grid origin is derived from the composition offset + global warp
        result_msg.info.origin.position.x = min_x * resolution
        result_msg.info.origin.position.y = min_y * resolution
        result_msg.info.origin.position.z = 0.0
        result_msg.info.origin.orientation.w = 1.0

        return result_msg

    def _ros_transform_to_matrix(self, t):
        # Convert quaternion to rotation matrix
        q = [t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w]
        
        # Simple yaw calculation from quaternion
        # (This is a simplified version of tf.transformations.euler_from_quaternion)
        x, y, z, w = q
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)

        cos_a = math.cos(yaw)
        sin_a = math.sin(yaw)

        # Pixel coordinates vs World coordinates conversion
        # The transform in params is typically in World Meters.
        # But we warp Pixels.
        # Note: The C++ implementation of GridWarper actually expects the transform 
        # to be applied to pixels. If inputs are meters, we must scale by resolution.
        # However, usually map merge assumes input transforms are consistent with grid indices
        # or it scales them. 
        # For this implementation, assuming params are in grid indices OR 
        # we strictly follow the logic that translation is divided by resolution outside.
        
        res = self.grids[0].info.resolution if self.grids else 0.05
        
        tx = t.translation.x / res
        ty = t.translation.y / res

        mat = np.array([
            [cos_a, -sin_a, tx],
            [sin_a, cos_a, ty],
            [0, 0, 1]
        ])
        return mat