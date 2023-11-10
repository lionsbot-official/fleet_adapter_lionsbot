import nudged
import math
import numpy as np

from typing import List

from .Coordinate import RmfCoord
from .Coordinate import LionsbotCoord

"""
    A wrapper class for transformation between a Lionsbot Robot Map and a RMF map 
    Transformation is done in coordinate frame => +X=right and +Y=up
    For accurate transformation, points must be in this coordinate frame

    +Y (m)
    ^
    |
    |
    |
    +-------------> +X (m)

    +Y (px)
    ^
    |
    |
    |
    +-------------> +X (px)
"""
class MapTransform:
    def __init__(self, 
                 floorplan_scale_meters_per_pixel: float,
                 robot_to_rmf_meters_transform: nudged.Transform, 
                 reference_to_target_meters_transform: nudged.Transform):
        
        self.floorplan_scale_meters_per_pixel = floorplan_scale_meters_per_pixel
        self.robot_to_rmf_meters_transform = robot_to_rmf_meters_transform
        self.rmf_meters_to_robot_transform = MapTransform._get_inverse_transform(robot_to_rmf_meters_transform.get_matrix())
        self.reference_to_target_meters_transform = reference_to_target_meters_transform
        self.target_meters_to_reference_transform = MapTransform._get_inverse_transform(reference_to_target_meters_transform.get_matrix())

    def compute(tx_meters: float, 
                 ty_meters: float, 
                 rotation_in_radians: float,
                 lbmap_translation_scale_factor: float, 
                 level_tx_pixels: float, 
                 level_ty_pixels: float,
                 floorplan_scale_meters_per_pixel: float):
                
        ts = math.cos(rotation_in_radians)*lbmap_translation_scale_factor
        tr = math.sin(rotation_in_radians)*lbmap_translation_scale_factor
        """
            In RMF Traffic Editor, the transform values are in coordinate frame => +X=right and +Y=down

                +-------------> +X 
                |
                |
                |
                v
                +Y

            To standardize the coordinate frame to be +X=right and +Y=up, we flip the y-axis
        """
        return MapTransform(floorplan_scale_meters_per_pixel=floorplan_scale_meters_per_pixel, 
                            robot_to_rmf_meters_transform=nudged.Transform(ts, tr, tx_meters, -ty_meters), 
                            reference_to_target_meters_transform=nudged.Transform(s=1, 
                                                                                  r=0, 
                                                                                  tx=-level_tx_pixels * floorplan_scale_meters_per_pixel, 
                                                                                  ty=level_ty_pixels * floorplan_scale_meters_per_pixel))
    
    def estimate(rmf_coords: List[List[float]], 
                 robot_coords: List[List[float]], 
                 floorplan_scale_meters_per_pixel: float, 
                 level_tx_pixels: float, 
                 level_ty_pixels: float):
        """
            In Lionsbot map, coordinate frame is => +X=right and +Y=down

                +-------------> +X (px)
                |
                |
                |
                v
                +Y (px)

            As transformation is done in the coordinate frame to be +X=right and +Y=up, we flip the y-axis
        """       
        for xy in robot_coords:
            xy[1] *= -1

        # standardize transform to be in reference level frame if applicable
        for xy in rmf_coords:
            xy[0] += level_tx_pixels * floorplan_scale_meters_per_pixel
            xy[1] -= level_ty_pixels * floorplan_scale_meters_per_pixel

        return MapTransform(floorplan_scale_meters_per_pixel=floorplan_scale_meters_per_pixel, 
                            robot_to_rmf_meters_transform=nudged.estimate(robot_coords, rmf_coords), 
                            reference_to_target_meters_transform=nudged.Transform(s=1, 
                                                                                  r=0, 
                                                                                  tx=-level_tx_pixels * floorplan_scale_meters_per_pixel, 
                                                                                  ty=level_ty_pixels * floorplan_scale_meters_per_pixel))
    
    def robot_to_rmf_meters(self, robot_coord: LionsbotCoord) -> RmfCoord:
        '''
            In Lionsbot map, coordinate frame is => +X=right and +Y=down

                +-------------> +X (px)
                |
                |
                |
                v
                +Y (px)

            As transformation is done in the coordinate frame to be +X=right and +Y=up, we flip the y-axis
        '''

        rmf_x_reference_frame, rmf_y_reference_frame = self.robot_to_rmf_meters_transform.transform([robot_coord.x, -robot_coord.y])

        # For maps whose rmf meters origin is that of another reference level, 
        # apply geometric transformation to shift origin to that of reference level
        rmf_coord = self._rmf_reference_meters_to_rmf_target_meters(
            RmfCoord(rmf_x_reference_frame, rmf_y_reference_frame))
        
        if robot_coord.orientation_radians is None:
            return rmf_coord
        else:
            return RmfCoord(
                x=rmf_coord.x, 
                y=rmf_coord.y, 
                orientation_radians=self._wrap_orientation(robot_coord.orientation_radians - self._get_orientation_offset()))
        
    
    def rmf_meters_to_robot(self, rmf_coord: RmfCoord) -> LionsbotCoord:
        """
            The result of transformation is in coordinate frame => +X=right and +Y=up

            +Y (px)
            ^
            |
            |
            |
            +-------------> +X (px)

            As coordinate frame is +X=right and +Y=down in Lionsbot map, we flip the y-axis
        """
        # For maps whose rmf meters origin is that of another reference level, 
        # apply geometric transformation to shift origin from reference level to this level
        rmf_coord_target_frame = self._rmf_target_meters_to_rmf_reference_meters(rmf_coord) 

        robot_x, robot_y = self.rmf_meters_to_robot_transform.transform([rmf_coord_target_frame.x, rmf_coord_target_frame.y])

        if rmf_coord.orientation_radians is None:
            return LionsbotCoord(robot_x, -robot_y)
        else:
            return LionsbotCoord(
                x=robot_x, 
                y=-robot_y, 
                orientation_radians=self._wrap_orientation(rmf_coord.orientation_radians + self._get_orientation_offset()))

        
    def _rmf_reference_meters_to_rmf_target_meters(self, rmf_coord: RmfCoord) -> RmfCoord:
        rmf_x, rmf_y = self.reference_to_target_meters_transform.transform([rmf_coord.x, rmf_coord.y])
        return RmfCoord(rmf_x, rmf_y)
    
    def _rmf_target_meters_to_rmf_reference_meters(self, rmf_coord: RmfCoord) -> RmfCoord:
        rmf_x, rmf_y = self.target_meters_to_reference_transform.transform([rmf_coord.x, rmf_coord.y])
        return RmfCoord(rmf_x, rmf_y)
    
    def _get_orientation_offset(self): 
        return self.robot_to_rmf_meters_transform.get_rotation()
    
    def _get_inverse_transform(matrix: list):
        np_matrix = np.array(matrix)
        inverse_matrix = np.linalg.inv(np_matrix)
        return nudged.Transform(
            inverse_matrix[0][0], inverse_matrix[1][0], inverse_matrix[0][2], inverse_matrix[1][2])
        
    # Ensure orientation is within range [-pi, pi]
    def _wrap_orientation(self, orientation_radians:float):
        if orientation_radians > np.pi:
            return orientation_radians - (2 * np.pi)
        elif orientation_radians < -np.pi:
            return orientation_radians + (2 * np.pi)
        else: 
            return orientation_radians
