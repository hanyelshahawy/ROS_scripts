#!/usr/bin/env python

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
# File name: main.py
# Author: Osama Abdelgawad
# E-mail: oabdelgawad@sigratech.de
# ***************************************************************************
# ***************************************************************************
# */

import sys
import rospy,os
from pub_tut.srv import *
 
def add_two_ints_client(x, y):
    while True:
        rospy.wait_for_service('add_two_ints')
        try:
            add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
            resp1 = add_two_ints(x, y)
            return resp1.sum
        except (rospy.ServiceException,e):
            print("Service call failed: %s"%e)

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 3:
        x = int(sys.argv[1])
        y = int(sys.argv[2])
    else:
        print(usage())
        sys.exit(1)
    while True:
        print("Requesting %s+%s"%(x, y))
        print("%s + %s = %s"%(x, y, add_two_ints_client(x, y)))
