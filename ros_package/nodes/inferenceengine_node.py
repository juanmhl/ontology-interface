#!/usr/bin/env python3

import rospy
from inferenceengine.srv import initialize
from inferenceengine import *
from std_msgs.msg import String
from std_msgs.msg import UInt8MultiArray


class InferenceEngineNode():

    allowed_predicates_vectors = [
        [True, False, False, False, False, False, True,  ],
        [True, False, False, False, True, False, True,   ],
        [True, False, False, False, True, True, False,   ],
        [True, True, False, False, True, True, False,    ],
        [False, True, False, False, True, True, False,   ],
        [False, True, False, True, True, False, False,   ],
        [False, True, True, True, False, False, False,   ],
        [True, True, True, True, False, False, False,    ],
        [True, False, True, True, False, False, False,   ],
    ]

    def __init__(self):
        self.num_stitches = 0
        self.initialized = False

        # Init node
        rospy.init_node('inferenceengine')

        # Init services
        s = rospy.Service('initialize', initialize, self.handle_initialize)
        rospy.loginfo("Ready to initialize inference engine")

        # Init subscribers
        rospy.Subscriber("predicates_vector", UInt8MultiArray, self.callback_predicates_vector, queue_size=1)

        # Init publisher, action topic is a string
        self.pub = rospy.Publisher('action', String, queue_size=10)

        # Spin at 50 Hz
        rate = rospy.Rate(50)
        rospy.spin()


    def handle_initialize(self,req):
        if (self.initialized == False) and (req.num_stitches > 0):
            # Get the number of stitches to initialize the inference engine
            self.num_stitches = req.num_stitches
            rospy.loginfo("Number of stitches: %d", self.num_stitches)

            # Initialize the inference engine
            repo_implementation = CommandRepositoryOntology     # Choose repo implementation
            self.service = InferenceMotorService(repo_implementation)
            self.service.initialize(n_points=self.num_stitches, ontology_path='/catkin_ws/src/inferenceengine/ontology.owl')
            self.initialized = True
            rospy.loginfo("Initialized inference engine")

            return True
        else:
            if self.initialized == True:
                rospy.loginfo("Inference engine already initialized")
            elif req.num_stitches <= 0:
                rospy.loginfo("Number of stitches must be greater than 0")
            return False

    def callback_predicates_vector(self,msg):

        if self.initialized == False:
            return
        else:
            # get the global service
           
            rospy.loginfo("Received predicates vector: %s", msg.data)

            # Test if the number of stitches corresponds with the expected length of the unformated predicates vector
            if len(msg.data) != (self.num_stitches + 7):
                rospy.loginfo("Length of predicates vector does not correspond with the number of stitches")
                return

            data_list = list(msg.data)
            data_list = [bool(x) for x in data_list]

            # Test if the predicates vector is allowed
            if data_list[0:7] not in self.allowed_predicates_vectors:
                rospy.loginfo("Predicates vector not allowed")
                return

            # Build the formated predicates vector from the data: list of bools. 
            # 7 first the same from data, 8th element is a list of the remainig data, with the state of each stitch
            predicates_vector = ([data_list[0],
                                data_list[1],
                                data_list[2],
                                data_list[3],
                                data_list[4],
                                data_list[5],
                                data_list[6],
                                data_list[7:]])
            rospy.loginfo("Predicates vector as a list: %s", predicates_vector)
            
            # Execute the step
            command = self.service.step(predicates_vector)

            # Publish the command and log it
            self.pub.publish(str(command))
            rospy.loginfo("Published command: %s", command)

            return

if __name__ == '__main__':
    try:
        InferenceEngineNode()
    except rospy.ROSInterruptException:
        pass
    