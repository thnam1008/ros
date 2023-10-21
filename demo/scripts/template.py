#!/usr/bin/python3
import rospy

class Template:
    def __init__(self):
        self.a=rospy.get_param("m/a","")

        def execute(sefl):
            print(sefl.a)
            
def main():
    rospy.init_node("template")
    template= Template()
    print(template.a)

main()