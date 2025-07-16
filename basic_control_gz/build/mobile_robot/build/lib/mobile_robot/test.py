###Rclpy es la libreeria de ros2 que se usa con cualquier instalacion estandar de ROS
### API para usar ros2 con python##

import rclpy


from rclpy.node import Node


from geometry_msgs.msg import Twist

topic1 = 'cmd_vel'  #topic al que nos suscribimos

rate_msg = 2 #2 mensajes por seg


def main(args=None):
    #Antes de usar la API hay que inicializarla con el init
    #Una vez por proceso, inicializa los recursos necesarios y el middleware.

    rclpy.init(args=args)

    #Twist message que estamos mandando, le podemos chantar velocidad lineal y angular

    controlVel = Twist()

    controlVel.linear.x=4.0;
    controlVel.linear.y=0.0;
    controlVel.linear.z=0.0;
    ##Componentes de la velocidad angular.
    controlVel.angular.x=0.0;
    controlVel.angular.y=0.0;
    controlVel.angular.z=8.0;
    
    testNode = Node("test_node")

    publisher = testNode.create_publisher(Twist,topic1,1)
    #create period rate for sending messages, here we are sending rate_msg msg per second.
    #Sending rate msg messages per second
    rate = testNode.create_rate(rate_msg)

    while rclpy.ok():
        #print message
        print("Sending control message")
        #Publish
        publisher.publish(controlVel)

        #Spin
        #Execute one item of work r wait until a timeout expires
        #This is a non blocking statement in contrast to rclpy.spin()
        rclpy.spin_once(testNode)
        #Sleep for the specified period
        rate.sleep()

    #Destroy the node
    testNode.destroy_node()
    #Shutdown
    rclpy.shutdown()

if __name__ == '__main__':
    main()