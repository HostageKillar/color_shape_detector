#Import the necessary libraries
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImagePublisher(Node):
    """
    Create an ImagePublisher class, which is a subclass of the Node class.
    """
    def __init__(self):
        """
        Class constructor to set up the node
        """
        #Initiate the Node class's constructor and give it a name
        super().__init__('image_publisher')

        #Create the publisher. This publisher will publish an Image
        #to the video_frams topic. The queue size is 10 messages.
        self.publisher_ = self.create_publisher(Image, 'video_frames', 10)

        #Will publish a message every 0.1 seconds
        time_period = 0.1   #seconds

        #Create the timer
        self.timer = self.create_timer(time_period, self.timer_callback)

        #Create a VideoCapture object
        #The argument '0' gets the default webcam.
        self.cap = cv2.VideoCapture(0)

        #Used to convert between ROS and OpenCV images
        self.br = CvBridge()


    def timer_callback(self):
        """
        Callback function.
        This function gets called every 0.1 seconds.
        """
        #Capture frame-by-frame
        #This method returns Tru/False as well
        #as the video frame.
        ret, frame = self.cap.read()

        if ret == True:
            #Publish the image.
            #The 'cv2_to_imgmsg' methode converts an OpenCV
            #image to a ROS2 image message
            self.publisher_.publish(self.br.cv2_to_imgmsg(frame))

        #Display the message on the console
        self.get_logger().info('Publishing video frame')

def main(args=None):

    #Initialize the rclpy library
    rclpy.init(args=args)

    #Create the node
    image_publisher = ImagePublisher()

    #Spin the node so the callback function is called
    rclpy.spin(image_publisher)

    #Destroy the node explicitly
    #(optional - otherwise it will be done automatically
    #when the garbege collector destroys the node object)
    image_publisher.destroy_node()

    #Shutdown the ROS client library for Python
    rclpy.shutdown()

if __name__ == '__main__':
    main()