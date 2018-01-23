#!/usr/bin python2.7

# /*
# ***************************************************************************
# ***************************************************************************
# (C) 2016 SIGRA Technologies GmbH  All rights reserved.
#
# All data and information contained in or disclosed by this document is
# confidential and proprietary information of SIGRA Technologies GmbH and all
# rights therein are expressly reserved.  By accepting this material the
# recipient agrees that this material and the information contained therein
# is held in confidence and in trust and will not be used, copied, reproduced
# in whole or in part, nor its contents revealed in any manner to others
# without the express written permission of SIGRA Technologies GmbH
#
# SIGRA Technologies GmbH
# Agnes-Pockels-Bogen 1,
# 80992, Munich,
# Germany
#
# File name: PublishNN.py
# Author: Hany Elshahawy
# E-mail: helshahawy@sigratech.de
# ***************************************************************************
# ***************************************************************************
# */

## file_description
# Test keras/tensorflow with python2.7
## end

import numpy as np
import rospy
from keras.models import load_model
from rospy_tutorials.msg import Floats

if __name__ == "__main__":
    try:
        # load NN
        model = load_model("demo.h5")
        # initialize node
        rospy.init_node('NN_Xcute', anonymous=True)
        # publish to topic
        NnPub = rospy.Publisher('NNcommands', Floats, queue_size=10)
        # rate of publication
        rate = rospy.Rate(10)  # 10hz
        while not rospy.is_shutdown():
            # execute NN
            res = model.predict_on_batch(np.ones((1, 150, 150, 3)))
            # publish to topic
            NnPub.publish(res)
            # print to log
            print 'NN commands: %s', res
            rate.sleep()
    except rospy.ROSInterruptException:
        print("Interrupted")



