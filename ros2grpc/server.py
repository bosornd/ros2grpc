from example_interfaces.srv import AddTwoInts

import rclpy
from rclpy.node import Node

from concurrent import futures

import grpc
from ros2grpc.calc_pb2 import Response
from ros2grpc.calc_pb2_grpc import AdderServicer, add_AdderServicer_to_server

class MinimalService(Node, AdderServicer):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))

        return response

    def add(self, request, context):
        self.get_logger().info('Incoming request through gRPC\na: %d b: %d' % (request.n1, request.n2))

        return Response(r = request.n1 + request.n2)

def main():
    rclpy.init()

    minimal_service = MinimalService()

    server = grpc.server(futures.ThreadPoolExecutor(max_workers=5))
    add_AdderServicer_to_server(minimal_service, server)

    server.add_insecure_port('[::]:50050')
    server.start()

    rclpy.spin(minimal_service)
    server.wait_for_termination()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
