from __future__ import annotations
import math
from typing import List

from src.models import CarPose, Cone, Path2D


class PathPlanning:
    """Student-implemented path planner.

    You are given the car pose and an array of detected cones, each cone with (x, y, color)
    where color is 0 for yellow (right side) and 1 for blue (left side). The goal is to
    generate a sequence of path points that the car should follow.

    Implement ONLY the generatePath function.
    """
    
    def __init__(self, car_pose: CarPose, cones: List[Cone]):
        self.car_pose = car_pose
        self.cones = cones

    def generatePath(self) -> Path2D:
        """Return a list of path points (x, y) in world frame.

        Requirements and notes: 
        - Cones: color==0 (yellow) are on the RIGHT of the track; color==1 (blue) are on the LEFT.
        - You may be given 2, 1, or 0 cones on each side.
        - Use the car pose (x, y, yaw) to seed your path direction if needed.
        - Return a drivable path that stays between left (blue) and right (yellow) cones.
        - The returned path will be visualized by PathTester.

        The path can contain as many points as you like, but it should be between 5-10 meters,
        with a step size <= 0.5. Units are meters.

        Replace the placeholder implementation below with your algorithm.
        """
        
        # Default: produce a short straight-ahead path from the current pose.
        # delete/replace this with your own algorithm.
        num_points = 25
        step = 0.5
        cx = self.car_pose.x
        cy = self.car_pose.y
        
        tyaw = self.car_pose.yaw

        def getdist(c):
            return math.hypot(c.x - cx, c.y - cy)
        
        blue_cones = [c for c in self.cones if c.color == 1 and getdist(c) > 1]
        yellow_cones = [c for c in self.cones if c.color == 0 and getdist(c) > 1]

        blue_cones.sort(key=getdist)
        yellow_cones.sort(key=getdist)

        points = [(cx,cy)]
        
        numpairs = min(len(blue_cones), len(yellow_cones))

        #cone pairs
        for i in range(numpairs):
            tx = (blue_cones[i].x + yellow_cones[i].x) / 2.0
            ty = (blue_cones[i].y + yellow_cones[i].y) / 2.0
            points.append((tx, ty))

        #blue cones
        for i in range(numpairs, len(blue_cones)):
            lastx, lasty = points[-1]
            angle_change = math.atan2(blue_cones[i].y - lasty, blue_cones[i].x - lastx)
            tx = blue_cones[i].x + 1.5 * math.cos(angle_change - math.pi / 2)
            ty = blue_cones[i].y + 1.5 * math.sin(angle_change - math.pi / 2)
            points.append((tx, ty))

        #yellow cones
        for i in range(numpairs, len(yellow_cones)):
            lastx, lasty = points[-1]
            angle_change = math.atan2(yellow_cones[i].y - lasty, yellow_cones[i].x - lastx)
            tx = yellow_cones[i].x + 1.5 * math.cos(angle_change + math.pi / 2)
            ty = yellow_cones[i].y + 1.5 * math.sin(angle_change + math.pi / 2)
            points.append((tx, ty))

        #No cones
        if len(points) == 1:
            tx = cx + 10.0 * math.cos(tyaw)
            ty = cy + 10.0 * math.sin(tyaw)
            points.append((tx, ty))

        #add a final waypoint ahead to satisfy the 5-10m rule
        if len(points) >= 2:
            #find the heading of the final segment
            final_yaw = math.atan2(points[-1][1] - points[-2][1], points[-1][0] - points[-2][0])
        else:
            final_yaw = tyaw
            
        extension_dist = 6.0 
        final_tx = points[-1][0] + extension_dist * math.cos(final_yaw)
        final_ty = points[-1][1] + extension_dist * math.sin(final_yaw)
        points.append((final_tx, final_ty))

        path: Path2D = []

        for i in range(len(points)-1):
            p1 = points[i]
            p2 = points[i+1]
            dist = math.hypot(p2[0] - p1[0], p2[1] - p1[1])

            steps = max(1, int(math.ceil(dist / step)))

            for j in range(steps):
                interpx = p1[0] + (p2[0] - p1[0]) * (j / steps)
                interpy = p1[1] + (p2[1] - p1[1]) * (j / steps)
                path.append((interpx, interpy))
                
        path.append(points[-1])
        
        return path
