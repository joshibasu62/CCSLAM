import cv2
import numpy as np

class GridWarper:
    def warp(self, grid, transform):
        """
        Warps a grid based on a transformation matrix.
        :param grid: numpy array (image)
        :param transform: 3x3 numpy array (homography/affine)
        :return: (warped_grid, roi_rect)
        """
        # Get the affine part of the 3x3 transform (2x3 matrix)
        affine_transform = transform[0:2, :]
        
        # Calculate the ROI (Region of Interest) / Bounding box
        roi_x, roi_y, roi_w, roi_h = self.warp_roi(grid, transform)
        
        # Adjust the translation in the affine matrix so the image fits 
        # inside the new bounding box (shift top-left to 0,0)
        # We need a copy to avoid modifying the original transform
        H_shifted = affine_transform.copy()
        H_shifted[0, 2] -= roi_x
        H_shifted[1, 2] -= roi_y

        # -1 is 'Unknown' in ROS occupancy grids (represented as signed int8)
        # We fill the border with -1 (255 in uint8 or -1 in int8)
        warped_grid = cv2.warpAffine(
            grid, 
            H_shifted, 
            (roi_w, roi_h), 
            flags=cv2.INTER_NEAREST, 
            borderMode=cv2.BORDER_CONSTANT, 
            borderValue=-1
        )

        return warped_grid, (roi_x, roi_y, roi_w, roi_h)

    def warp_roi(self, grid, transform):
        """
        Calculates the bounding box of the warped image.
        """
        h, w = grid.shape
        corners = np.array([
            [0, 0],
            [w, 0],
            [w, h],
            [0, h]
        ], dtype=np.float32)

        # Reshape for cv2.transform: (N, 1, 2)
        corners = np.array([corners])
        
        # We need the affine part for cv2.transform
        affine_transform = transform[0:2, :]
        
        # Transform corners
        warped_corners = cv2.transform(corners, affine_transform)
        
        # Calculate bounding rect
        x, y, w, h = cv2.boundingRect(warped_corners)
        
        return x, y, w, h