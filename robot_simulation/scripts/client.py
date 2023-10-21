import rospy
from robot_msgs.srv import ChangeColor, ChangeColorRequest, ChangeColorResponse
from std_msgs.msg import ColorRGBA


class Client:
    def __init__(self):
        self.client = rospy.ServiceProxy("change_color", ChangeColor)

        self.color_list = [
            ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0),
            ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0),
            ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0),
            ColorRGBA(r=0.0, g=1.0, b=1.0, a=1.0),
            ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0),
            ColorRGBA(r=1.0, g=0.0, b=1.0, a=1.0),
        ]
        rospy.loginfo("Init client Done")

    def execute(self):
        color_idx = 0
        while not rospy.is_shutdown():
            req = ChangeColorRequest()
            req.color = self.color_list[color_idx % 6]
            rospy.loginfo("Create request done")

            res = self.client(req)
            rospy.loginfo("Call service done")
            color_idx += 1
            rospy.sleep(1.0)


def main():
    rospy.init_node("change_color_client")
    client = Client()
    client.execute()


main()