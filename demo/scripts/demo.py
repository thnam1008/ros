import rospy

def main():
    rospy.init_node("demo_python")

    m_a = rospy.get_param("m/c","default")

    print(m_a)
    while not rospy.is_shutdown():
        pass
    print("End demo")
main()
