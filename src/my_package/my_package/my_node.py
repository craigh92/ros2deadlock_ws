import rclpy
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from rclpy.task import Future
from rclpy.waitable import Waitable
import os
import argparse
import threading

latching_qos = QoSProfile(depth=1,
    durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)

def main():

    # Parse args

    parser = argparse.ArgumentParser()
    parser.add_argument('--exe', choices=['M','S'], required=True,
        help='The Executor to use. M for MultiThreaded or S for SingleThreaded')
    parser.add_argument('--d', action='store_true',
        help='Print a debug message at the start and end of every spin')

    args = parser.parse_args()

    # Init ros

    rclpy.init()
    exe = MultiThreadedExecutor() if args.exe == 'M' else SingleThreadedExecutor()

    # Set up publisher

    pubnode = Node('pubnode_' + str(os.getpid()))
    
    pub1 = pubnode.create_publisher(String, 'topic1', latching_qos)
    msg1 = String()
    msg1.data = "hello1"
    pubnode.get_logger().info("Publishing hello1: Thread {}".format(threading.current_thread().name))
    pub1.publish(msg1)


    # Set up listener

    future_msgs = Future()

    subnode = Node('subnode_' + str(os.getpid()))
    subnode.create_subscription(String, 'topic1', lambda msg : ([

            subnode.get_logger().info("Received message on topic1: Thread {}".format(threading.current_thread().name)),
            future_msgs.set_result(msg),
            subnode.get_logger().info("Callback finished")

    ]), latching_qos)

    # Start nodes

    print("Using {}".format(str(type(exe))))
    exe.add_node(pubnode)
    exe.add_node(subnode)
    
    if args.d:
        while future_msgs.done() is False:
            print('------ Begin Spin : {}'.format(threading.current_thread().name))
            exe.spin_once()
            print('------ End Spin')
    else:
        exe.spin_until_future_complete(future_msgs)


    print("Goodbye!")

    pubnode.destroy_node()
    subnode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
