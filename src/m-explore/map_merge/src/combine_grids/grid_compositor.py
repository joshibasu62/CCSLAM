import numpy as np
import cv2
from nav_msgs.msg import OccupancyGrid

class GridCompositor:
    def compose(self, grids, rois):
        """
        Composes multiple grids into one based on their ROIs.
        :param grids: list of numpy arrays (warped maps)
        :param rois: list of tuples (x, y, w, h)
        :return: nav_msgs.msg.OccupancyGrid
        """
        if len(grids) != len(rois) or len(grids) == 0:
            return None

        # 1. Calculate the global bounding box
        min_x = float('inf')
        min_y = float('inf')
        max_x = float('-inf')
        max_y = float('-inf')

        for (x, y, w, h) in rois:
            min_x = min(min_x, x)
            min_y = min(min_y, y)
            max_x = max(max_x, x + w)
            max_y = max(max_y, y + h)

        width = int(max_x - min_x)
        height = int(max_y - min_y)

        # 2. Allocate result grid filled with -1 (Unknown)
        # Using int8 to match ROS OccupancyGrid data type
        result_img = np.full((height, width), -1, dtype=np.int8)

        # 3. Paste grids
        for i, grid in enumerate(grids):
            roi_x, roi_y, roi_w, roi_h = rois[i]
            
            # Calculate offset in the result image
            offset_x = roi_x - min_x
            offset_y = roi_y - min_y

            # Extract the slice from the result image where we want to paste
            target_slice = result_img[offset_y:offset_y+roi_h, offset_x:offset_x+roi_w]
            
            # Resize grid if necessary (should match roi, but safety check)
            if grid.shape != target_slice.shape:
                continue

            # Merge logic: np.maximum
            # In int8: 100 > 0 > -1. 
            # So Obstacles > Free > Unknown.
            result_img[offset_y:offset_y+roi_h, offset_x:offset_x+roi_w] = np.maximum(target_slice, grid)

        # 4. Construct ROS Message
        msg = OccupancyGrid()
        msg.info.width = width
        msg.info.height = height
        
        # Flatten data for ROS message
        msg.data = result_img.flatten().tolist()
        
        return msg, min_x, min_y