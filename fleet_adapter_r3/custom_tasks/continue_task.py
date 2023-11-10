import sys
import uuid
import argparse
import json

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy as History
from rclpy.qos import QoSDurabilityPolicy as Durability
from rclpy.qos import QoSReliabilityPolicy as Reliability

from rmf_task_msgs.msg import ApiRequest


###############################################################################

class TaskRequester(Node):

    def __init__(self, argv=sys.argv):
        super().__init__('task_requester')
        parser = argparse.ArgumentParser()
        parser.add_argument('-R', '--robot_name', required=True,
                            type=str, help='Robot name')
        parser.add_argument('-F', '--fleet', required=True,
                            type=str, help='Fleet name')

        self.args = parser.parse_args(argv[1:])

        transient_qos = QoSProfile(
            history=History.KEEP_LAST,
            depth=1,
            reliability=Reliability.RELIABLE,
            durability=Durability.TRANSIENT_LOCAL)

        self.pub = self.create_publisher(
          ApiRequest, 'lb_task_api_requests', transient_qos)

        # Construct task
        msg = ApiRequest()
        msg.request_id = "continue_task_" + str(uuid.uuid4())
        payload = {}
        payload["type"] = "continue_task_request"
        payload["robot_name"] = self.args.robot_name
        payload['fleet'] = self.args.fleet
        # payload["task_id"] = self.args.task_id

        msg.json_msg = json.dumps(payload)
        print(f"msg: \n{json.dumps(payload, indent=2)}")
        self.pub.publish(msg)

###############################################################################


def main(argv=sys.argv):
    rclpy.init(args=sys.argv)
    args_without_ros = rclpy.utilities.remove_ros_args(sys.argv)

    task_requester = TaskRequester(args_without_ros)
    rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)