import sys
from datetime import datetime

from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

import grpc
from ros2grpc.calc_pb2 import Request
from ros2grpc.calc_pb2_grpc import AdderStub

class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main():
    rclpy.init()

    minimal_client = MinimalClientAsync()
    start_time = datetime.now()
    results = [ minimal_client.send_request(i, i + 1).sum for i in range(100) ]
    time_elasped = datetime.now() - start_time

    print(results)
    print('time elasped(ROS) : {}'.format(time_elasped))

    channel = grpc.insecure_channel('localhost:50050')
    stub = AdderStub(channel)
    start_time = datetime.now()
    results = [ stub.add(Request(n1 = i, n2 = i + 1)).r for i in range(100) ]
    time_elasped = datetime.now() - start_time

    print(results)
    print('time elasped(gRPC) : {}'.format(time_elasped))

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
