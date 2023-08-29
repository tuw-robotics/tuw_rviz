#!/usr/bin/env python3
#
# Copyright (c) 2023 Markus Bader <markus.bader@tuwien.ac.at>
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from math import pi, sin, cos
from numpy import array
from numpy.linalg import norm

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from tuw_multi_robot_msgs.msg import Graph, Vertex
from geometry_msgs.msg import Point


def quaternion_about_axis(angle, axis):
    axis = array(axis)
    axis = axis / norm(axis)
    half_angle = angle / 2
    sine = sin(half_angle)
    w = cos(half_angle)
    x, y, z = axis * sine
    return x, y, z, w


def create_point(x, y):
    point = Point()
    point.x = x
    point.y = y
    point.z = 0.
    return point

def create_vertex(id, points, successors, predecessors):
    vertex = Vertex()
    vertex.valid = True
    for p in points:
        vertex.path.append(create_point(p[0],p[1]))
    for v in successors:
        vertex.successors.append(v)
    for v in predecessors:
        vertex.predecessors.append(v)
    return vertex
    
class publish_graph(Node):
    def __init__(self):
        super().__init__('publish_graph_sample')
        self.__pub = self.create_publisher(
            Graph, 'graph', 10)
        self.__timer = self.create_timer(0.1, self.pub_sample)
        self.__counter = 0
        self.__header = Header()


    def pub_sample(self):
        while self.__pub.get_subscription_count() == 0:
            return
        self.__header.stamp = self.get_clock().now().to_msg()
        self.__header.frame_id = 'map'

        # Reset counter and indices if counter is a multiple of 30
        if self.__counter % 10 == 0:
            self.__counter = 0

        # Create a single Graph message
        graph = Graph()
        graph.origin.position.x = 0.
        graph.origin.position.y = 0.
        graph.origin.position.z = 0.
        graph.origin.orientation.x = 0.
        graph.origin.orientation.y = 0.
        graph.origin.orientation.z = 0.
        graph.origin.orientation.w = 1.

        graph.vertices.append(create_vertex(0, [[-6., -6.], [-5., -7.], [-4., -7.], [-3., -6.]], [1,6],[3]))
        graph.vertices.append(create_vertex(1, [[-3., -6.], [-2., -2.], [-3., +1.]], [2,4],[0,6]))
        graph.vertices.append(create_vertex(2, [[-3., +1.], [-5.,  0.], [-6.,  0.]], [3],[1,4]))
        graph.vertices.append(create_vertex(3, [[-6.,  0.], [-6., -6.]], [0],[2]))
        graph.vertices.append(create_vertex(4, [[-3.,  1.], [-1., -1.], [ 3., -2.]], [5],[1,2]))
        graph.vertices.append(create_vertex(5, [[ 3., -2.], [ 4., -3.], [ 3., -5.]], [6],[4]))
        graph.vertices.append(create_vertex(6, [[ 3., -5.], [ 0., -6.], [-3., -6.]], [0,1],[5]))

        # Publish the BoundingBox3D message
        graph.header = self.__header
        self.__pub.publish(graph)
        self.__counter += 1


def main(args=None):
    rclpy.init(args=args)
    node = publish_graph()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
