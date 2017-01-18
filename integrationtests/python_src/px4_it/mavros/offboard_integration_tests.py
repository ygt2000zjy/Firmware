#!/usr/bin/env python
#***************************************************************************
#
#   Copyright (c) 2015 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
#***************************************************************************/

#
# @Dennis Mannhart <dennis.mannhart@gmail.com>
#
# main file that calls single tests depending on input
# 
##################### INFO ############################
# To run tests locally, SITL and mavros must be running
# To load single test 
#    ./offboard_integration_tests -i name_of_test
# To do all tests available:
#    ./offboard_integration_tests
# The list of all tests are in tests/tests_list.txt file
########################################################

from __future__ import print_function

import sys
import getopt
from os import path
import argparse

from tests import run_tests


"""
Starts integration tests
"""


def usage():
    print(">> usage: offboard_integration_tests.py -i [AVAILABLE TEST]")


def main(argv):

    tests_list_file = path.relpath("tests/tests_list.txt")
    
    try:
        opts, args = getopt.getopt(argv, "hi: ")
    except getopt.GetoptError: 
        print("This option is not supported")
        usage()
        return
        
    
    # open list with tests
    with open(tests_list_file) as f:
        tests_available = f.read().splitlines()
    
    
    do_tests = 0
    # no argument given: run all tests
    if opts == []:
        print("Run all tests")
        do_tests = tests_available
    
   
    # get specific test
    for opt, arg in opts:

        if opt in ('-h', '--help'):
            usage()
            sys.exit()
            
        elif opt in ('-i', '--i'):
            
            if arg in tests_available:
                do_tests = [arg]
                
        else:
            print("Input not supported")
            usage()
            return 
    
    # desired test does not exist  
    if not do_tests :
        print(">> This test does not exists \nAvailable tests:")
        for t in tests_available: print(t)
        return
        
    """ Run Tests """
    run_tests(do_tests)


if __name__ == '__main__':
    main(sys.argv[1:])





