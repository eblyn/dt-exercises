#!/usr/bin/env python3
import numpy as np
import rospy

from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from duckietown_msgs.msg import Twist2DStamped, LanePose, WheelsCmdStamped,BoolStamped, FSMState, StopLineReading, Segment, SegmentList

from lane_controller.controller import PurePursuitLaneController


class LaneControllerNode(DTROS):
    """Computes control action.
    The node compute the commands in form of linear and angular velocitie.
    The configuration parameters can be changed dynamically while the node is running via ``rosparam set`` commands.
    Args:
        node_name (:obj:`str`): a unique, descriptive name for the node that ROS will use
    Configuration:

    Publisher:
        ~car_cmd (:obj:`Twist2DStamped`): The computed control action
    Subscribers:
        ~lane_pose (:obj:`LanePose`): The lane pose estimate from the lane filter
    """

    def __init__(self, node_name):

        # Initialize the DTROS parent class
        super(LaneControllerNode, self).__init__(
            node_name=node_name,
            node_type=NodeType.CONTROL
        )

        # Add the node parameters to the parameters dictionary
        self.params = dict()
        self.pp_controller = PurePursuitLaneController(self.params)
        self.omega = 0

        # Construct publishers
        self.pub_car_cmd = rospy.Publisher("~car_cmd",
                                           Twist2DStamped,
                                           queue_size=1,
                                           dt_topic_type=TopicType.CONTROL)

        # Construct subscribers
        self.sub_segments = rospy.Subscriber("~/agent/ground_projection_node/lineseglist_out",
                                                 SegmentList,
                                                 self.cbSegments,
                                                 queue_size=1)

        # self.sub_lane_reading = rospy.Subscriber("~lane_pose",
        #                                          LanePose,
        #                                          self.cbLanePoses,
        #                                          queue_size=1)
                                                

        self.log("Initialized!")


    def cbSegments(self, input_segments_msg):
        """Callback receiving pose messages

        Args:
            input_pose_msg (:obj:`LanePose`): Message containing information about the current lane pose.
        """
        self.segment_msg = input_segments_msg
        car_control_msg = Twist2DStamped()
        car_control_msg.header = self.segment_msg.header

        (v,w) = self.pp_controller.pure_pursuit_from_segments(self.segment_msg)
        # TODO This needs to get changed
        car_control_msg.v = v

        # alpha = 0.4
        # w_amortized = (w + self.omega)/2
        # car_control_msg.omega = w_amortized
        # self.omega = w_amortized
        
        car_control_msg.omega = w
        self.publishCmd(car_control_msg)


    def cbLanePoses(self, input_pose_msg):
        """Callback receiving pose messages

        Args:
            input_pose_msg (:obj:`LanePose`): Message containing information about the current lane pose.
        """
        self.pose_msg = input_pose_msg
        # print(self.pose_msg)
        car_control_msg = Twist2DStamped()
        car_control_msg.header = self.pose_msg.header

        (v,w) = self.pp_controller.pure_pursuit_from_pose(self.pose_msg.d, self.pose_msg.phi)
        # TODO This needs to get changed
        car_control_msg.v = v
        
        alpha = 0.6
        w_amortized = (w + self.omega)/2
        car_control_msg.omega = w_amortized
        self.omega = w_amortized

        self.publishCmd(car_control_msg)




    def publishCmd(self, car_cmd_msg):
        """Publishes a car command message.

        Args:
            car_cmd_msg (:obj:`Twist2DStamped`): Message containing the requested control action.
        """
        self.pub_car_cmd.publish(car_cmd_msg)


    def cbParametersChanged(self):
        """Updates parameters in the controller object."""

        self.controller.update_parameters(self.params)


if __name__ == "__main__":
    # Initialize the node
    lane_controller_node = LaneControllerNode(node_name='lane_controller_node')
    # Keep it spinning
    rospy.spin()
