import numpy as np
from duckietown_msgs.msg import Segment

# import debugpy
# debugpy.listen(("localhost", 5678))


class PurePursuitLaneController:
    """
    The Lane Controller can be used to compute control commands from pose estimations.

    The control commands are in terms of linear and angular velocity (v, omega). The input are errors in the relative
    pose of the Duckiebot in the current lane.

    """

    def __init__(self, parameters):

        self.parameters = parameters

    def update_parameters(self, parameters):
        """Updates parameters of LaneController object.

            Args:
                parameters (:obj:`dict`): dictionary containing the new parameters for LaneController object.
        """
        self.parameters = parameters

    def pure_pursuit_from_segments(self, segments_msg, L=1.5, d=1):        
        segments = np.array(segments_msg.segments)
        # print(segments)
        S_yellow = self.closest_color(segments, L, Segment.YELLOW)

        S = [s for s in segments if s.points[0].y <= 0]
        
        if S_yellow:
            S = [s for s in S if self.right_lane(s, S_yellow)]
        
        S_white = self.closest_color(S, L, Segment.WHITE)
        if S_yellow:
            return self.pure_pursuit_yellow(S_yellow, d)
        if S_white:
            return self.pure_pursuit_white(S_white, d)
        return self.pure_pursuit_middle(S_white, S_yellow)

    def right_lane(self, segment, S_yellow):
        (p1, p2) = S_yellow.points
        m = (p1.y - p2.y)/(p1.x - p2.x)
        n = p1.y - m*p1.x
        (x, y) = self.segment_center(segment)
        return np.sign(m)*(y - m*x - n) >= 0

    def pure_pursuit_middle(self, S_white, S_yellow):

        look_ahead = (self.segment_center(S_white) + self.segment_center(S_yellow))/2
        (x,_) = look_ahead
        sin_alpha = x/self.dist(look_ahead)
        v = 0.1 # we can make it constant or we can make it as a function of sin_alpha
        K = float(self.dist(look_ahead)/v)
        w = sin_alpha/K
        return (v, w)

    def pure_pursuit_white(self, S_white, d):
         # get perpendicular to the center
        seg_center = self.segment_center(S_white)
        (p1, p2) = S_white.points
        seg_length = np.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2) 
        # add the distance to the center in perpendicular
        look_ahead = seg_center - 2*d/seg_length*self.d_coord(p2, seg_center)
        (x,_) = look_ahead
        sin_alpha = x/self.dist(look_ahead)
        v = 0.1 # we can make it constant or we can make it as a function of sin_alpha
        K = float(self.dist(look_ahead)/v)
        w = sin_alpha/K
        return (v, w)

    def pure_pursuit_yellow(self, S_yellow, d):
         # get perpendicular to the center
        seg_center = self.segment_center(S_yellow)
        (p1, p2) = S_yellow.points
        seg_length = np.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2) 
        # add the distance to the center in perpendicular
        look_ahead = seg_center - 2*d/seg_length*self.d_coord(p2, seg_center)
        (x,_) = look_ahead
        sin_alpha = x/self.dist(look_ahead)
        v = 0.1 # we can make it constant or we can make it as a function of sin_alpha
        K = float(self.dist(look_ahead)/v)
        w = sin_alpha/K
        return (v, w)

    def d_coord(self, p, c):
        return np.array([p.y - c[1], p.x - c[0]])

    def closest_color(self, segments, L, color):
        seg_color = [s for s in segments if s.color == color]
        if len(seg_color) == 0:
            return None
        centers = [self.segment_center(seg) for seg in seg_color]
        distances = [(self.dist(c) - L) for c in centers]
        return seg_color[np.argmin(distances)]

    def segment_center(self, seg):
        (p1, p2) = seg.points
        return np.array([(p1.x + p2.x)/2, (p1.y + p2.y)/2])
    
    def dist(self,X):
        return np.sqrt(np.sum(np.array(X)**2))

    def pure_pursuit_from_pose(self, d, phi, L=1):
        """
        In this method we are asuming that the lane is straight, just a dumb pure pursuit to start
        Input:
            - d: distance from the center of the lane.
            - phi: angle $phi$ with respect to the center of the lane.
            - L: distance between robot and follow point in the lane
        Return:
            - v: linear velocity in m/s (float)
            - w: angular velocity in rad/s (float)
        """
        beta = np.arccos(float(d/L))
        sin_alpha = np.sin(np.pi/2 + phi - beta)
        
        v = 0.1 # we can make it constant or we can make it as a function of sin_alpha
        K = float(L/v)
        # angular velocity
        w = sin_alpha/K
        
        return v, w
