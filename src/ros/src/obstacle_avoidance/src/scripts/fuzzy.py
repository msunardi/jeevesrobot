#!#/bin/bash
"""
    Fuzzy logic stuff

    1. Define membership functions for each input and output
    2. Define rules
    3. get input
    4. input map to membership functions
    5. 
    
"""

import os
import numpy as np

from math import *


class Membership:

    def __init__(self, name):
        self.name = name
        self.membership_functions = {}  # dictionary of MembershipFunction

    def add_membership(self, key):
        self.membership_functions[key] = []

    def add_membership_func(self, key, membership_func):
        self.membership_functions[key] = membership_func

    def get_membership(self, value, input_mem_func=None):
        #print self.name
        output = {}
        if not input_mem_func:
            # Use all in self.input_membership_functions
            # returns dictionary: { membership_name: degree_of_membership }
            for key, mem in self.membership_functions.iteritems():
                #print key
                output[key] = mem.get_value(value)
                self.fuzzified_input = output
                
        else:
            # Assume, if given, only apply the specific membership function
            # Returns degree of membership for the given value and membership function
            output = input_mem_func.get_value(value)
        return output
        

class MembershipFunction:
    """
        Format: (val, membership), (val, membership), (val, membership)
        e.g. (0, 1), (10, 1), (15, 0)
             (10, 0), (15, 1), (20, 0)
             
    """
    def __init__(self, mem_ship):
        self.mem_ship = mem_ship

    def get_value(self, input_val):
        ms_count= len(self.mem_ship)
        for i in range(ms_count - 1):
            #print "input_val: %s" % (input_val)
            #print "i: %s, mem_ship1: %s | mem_ship2: %s" % (i, self.mem_ship[i], self.mem_ship[i+1])
            if (i==0 and input_val <= self.mem_ship[0][0]):
                #print "case 1"
                return float(self.mem_ship[i][1])
            
            if input_val >= self.mem_ship[i][0] and input_val < self.mem_ship[i+1][0]:
                #print "case 3"
                return self.member(self.mem_ship[i], self.mem_ship[i+1], input_val)

        else:
            return float(self.mem_ship[i+1][1])
            
        
    def member(self, mem1, mem2, value):
        """
            mem# = (value, membership)
        """
        if mem1[1] == 1 and mem2[1] == 1:
            return 1.0
        elif mem1[1] == 0 and mem2[1] == 1: # on incline slope
            return float(value-mem1[0])/(mem2[0] - mem1[0])
        else:   # on decline slope
            return 1- float(value-mem1[0])/(mem2[0] - mem1[0])

class Fuzzy(object):

    def __init__(self):
        self.input_membership_functions = {}    # Dictionary of Membership
        self.output_membership_functions = {}   # Dictionary of Membership
        self.rules = []
        self.fuzzified_input = {}

    def add_input_membership(self, input_name):
        new_member = Membership(input_name)
        self.input_membership_functions[input_name] = new_member

    def add_input_membership_func(self, input_name, var_name, membership_func):
        #self.input_membership_functions[input_name].add_membership(var_name)
        self.input_membership_functions[input_name].add_membership_func(var_name, membership_func)

    def add_output_membership(self, output_name):
        new_member = Membership(output_name)
        self.output_membership_functions[output_name] = new_member

    def add_output_membership_func(self, output_name, var_name, membership_func):
        #self.output_membership_functions[output_name].add_membership(var_name)
        self.output_membership_functions[output_name].add_membership_func(var_name, membership_func)    

    def add_rule(self, *args, **kwargs):
        """
            Usage: inputs use args (by default AND), output specify in kwargs
            e.g. "If a AND b AND c then output = x
            a,b,c are membership function names
            add_rule(a,b,c, output=x)
        """
        #self.rules.append({'input':args, 'output':kwargs['output']})

    def add_rule2(self, rule_name, rule_function):
        self.rules.append({rule_name: rule_function})
                          
    def all_input_membership(self, input_value):
        """
            Don't use this
        """
        return {key: member.get_membership(input_value) for key, member in self.input_membership_functions.iteritems()}

    def get_membership_for(self, input_type, input_value):
        """
            Returns the degree of membership for all the membership functions in the input_type for the given input_value
        """
        return self.input_membership_functions[input_type].get_membership(input_value)
    

    #def evaluate_rules(self):

def fuzzyAND(input1, input2):
    return min(input1, input2)

def fuzzyOR(input1, input2):
    return max(input1, input2)

def fuzzyNOT(input_value):
    return 1.0 - input_value

def main():
    """
        Test:
            1. Create a set of membership functions.
            2. Add a few rules
            3. Calculate which rules fire, and give values
    """

    left_near = MembershipFunction( ((0.0,1.0), (1.5,1.0), (2.0,0.0)) )
    left_mid = MembershipFunction( ((1.5,0.0), (2.0,1.0), (3.5,1.0), (4.0,0.0)) )
    left_far = MembershipFunction( ((3.5,0.0), (4.0,1.0), (5.0,1.0)) )

    front_near = MembershipFunction( ((0.0,1.0), (2.0,1.0), (2.5,0.0)) )
    front_mid = MembershipFunction( ((2.0,0.0), (2.5,1.0), (4.0,1.0), (4.5,0.0)) )
    front_far = MembershipFunction( ((4.0,0.0), (4.5,1.0),(6.0,1.0)) )

    right_near = MembershipFunction( ((0.0,1.0), (1.5,1.0), (2.0,0.0)) )
    right_mid = MembershipFunction( ((1.5,0.0), (2.0,1.0), (3.5,1.0), (4.0,0.0)) )
    right_far = MembershipFunction( ((3.5,0.0), (4.0,1.0), (5.0,1.0)) )

    output_turn_left = MembershipFunction( ((-45,1),(-22.5,1),(0,0)) )
    output_no_turn = MembershipFunction( ((-22.5,0),(0,1),(22.5,0)) )
    output_turn_right = MembershipFunction( ((0,0),(22.5,1),(45,1)) )

    heading_delta_right = MembershipFunction( ((-90,1),(-15,1),(0,0)) )
    heading_delta_front = MembershipFunction( ((-15,0),(0,1),(15,0)) )
    heading_delta_left = MembershipFunction( ((0,0),(15,1),(90,1)) )

    fuzz = Fuzzy()
    fuzz.add_input_membership('left')
    fuzz.add_input_membership_func('left','near', left_near)
    fuzz.add_input_membership_func('left','mid', left_mid)
    fuzz.add_input_membership_func('left','far', left_far)

    left = fuzz.get_membership_for('left',40)
    print "Left: %s" % left
    
    fuzz.add_input_membership('front')
    fuzz.add_input_membership_func('front','near', front_near)
    fuzz.add_input_membership_func('front','mid', front_mid)
    fuzz.add_input_membership_func('front','far', front_far)

    front = fuzz.get_membership_for('front', 40)
    print "Front: %s" % front

    fuzz.add_input_membership('right')
    fuzz.add_input_membership_func('right','near', right_near)
    fuzz.add_input_membership_func('right','mid', right_mid)
    fuzz.add_input_membership_func('right','far', right_far)

    right = fuzz.get_membership_for('right', 40)
    print "Right: %s" % right

    fuzz.add_input_membership('heading_delta')
    fuzz.add_input_membership_func('heading_delta','right',heading_delta_right)
    fuzz.add_input_membership_func('heading_delta','front',heading_delta_front)
    fuzz.add_input_membership_func('heading_delta','left',heading_delta_left)
    
    turn_right = fuzzyAND(fuzzyAND(front['near'], left['near']), fuzzyNOT(right['near']))**2 +\
                 fuzzyAND(fuzzyNOT(right['near']), fuzzyAND(left['mid'], front['mid']))**2
    
    go_forward = fuzzyNOT(front['near'])**2 +\
                 fuzzyAND(fuzzyNOT(front['near']), fuzzyAND(left['near'],right['near']))**2 +\
                 fuzzyAND(fuzzyNOT(front['near']), fuzzyAND(left['near'], fuzzyNOT(right['near'])))**2
                          
    turn_left = fuzzyAND(fuzzyAND(front['near'],right['near']), fuzzyNOT(left['near']))
                          
    print "Turn_right: %s" % turn_right
    print "Go_forward: %s" % go_forward
    print "Turn_left: %s" % turn_left

    if turn_right > 0.0 or turn_left > 0.0:
        output = (sqrt(turn_right)*-45 + sqrt(go_forward)*1 + sqrt(turn_left)*45)/(sqrt(turn_right) + sqrt(go_forward) + sqrt(turn_left))
    else:
        output = 0.0

    print "Output (turn degree): %s" % output

    scenario1 = [(100,100,100),(100,100,100),(100,100,100),(100,100,100),(100,90,100),(100,80,90),(100,70,80),(100,60,70),\
                 (100,55,65),(100,50,60),(100,45,55),(100,42,52),(100,40,50),(100,30,40),(100,20,30)]

    scenario2 = [(100,100,100),(100,100,100),(100,100,100),(100,100,100),(100,90,100),(80,90,100),(70,80,100),(60,70,100),\
                 (55,65,100),(50,60,100),(45,55,100),(42,52,100),(40,50,100),(30,40,100),(20,30,100)]

    scenario3 = [(100,100,100),(100,100,100),(100,100,100),(100,100,100),(100,90,100),(80,90,100),(70,80,100),(60,70,100),\
                 (55,65,100),(50,60,100),(45,55,100),(42,52,60),(40,50,52),(37,46,48),(34,42,40),(20,30,80)]

    scenario4 = [(100,100,100,0),(100,100,100,0),(100,100,100,0),(100,100,100,-5),(100,90,100,-8),(80,90,100,-10),(70,80,100,-15),\
                 (60,70,100,-20),(55,65,100,-17),(50,60,100,-14),(45,55,100,-5),(42,52,60,0),(40,50,52,8),(37,46,48,12),(34,42,40,18),\
                 (20,30,80,16)]

    for s in scenario4:
        print s
        left = fuzz.get_membership_for('left',s[0]/10.0)
        right = fuzz.get_membership_for('right',s[2]/10.0)
        front = fuzz.get_membership_for('front',s[1]/10.0)
        heading = fuzz.get_membership_for('heading_delta',s[3]) # Use in scenario4+

        print "Left: %s" % left
        print "Right: %s" % right
        print "Front: %s" % front
        print "Heading offset: %s" % heading

        turn_right = fuzzyAND(fuzzyNOT(right['near']), fuzzyAND(front['near'], left['near']))**2\
                     + fuzzyAND(fuzzyNOT(right['near']), fuzzyAND(front['near'], left['mid']))**2\
                     + fuzzyAND(fuzzyNOT(right['near']), fuzzyAND(front['mid'], left['near']))**2\
                     + fuzzyAND(fuzzyNOT(right['near']), fuzzyAND(front['mid'], left['mid']))**2\
                     + fuzzyAND(fuzzyNOT(right['near']), heading['left'])**2
         
        go_forward = fuzzyNOT(front['near'])**2\
                     + fuzzyAND(fuzzyNOT(front['near']), fuzzyAND(left['near'],right['near']))**2\
                     + fuzzyAND(fuzzyNOT(front['near']), fuzzyAND(left['near'], fuzzyNOT(right['near'])))**2
                              
        turn_left = fuzzyAND(fuzzyNOT(left['near']), fuzzyAND(right['near'],front['near']))**2\
                    + fuzzyAND(fuzzyNOT(left['near']), fuzzyAND(right['near'], front['mid']))**2\
                    + fuzzyAND(fuzzyNOT(left['near']), fuzzyAND(right['mid'], front['near']))**2\
                    + fuzzyAND(fuzzyNOT(left['near']), fuzzyAND(right['mid'], front['mid']))**2\
                    + fuzzyAND(fuzzyNOT(left['near']), heading['right'])**2
                          
        print "Turn_right: %s" % turn_right
        print "Go_forward: %s" % go_forward
        print "Turn_left: %s" % turn_left

        if turn_right > 0.0 or turn_left > 0.0:
            output = (sqrt(turn_right)*-45 + sqrt(go_forward)*1 + sqrt(turn_left)*45)/(sqrt(turn_right) + sqrt(go_forward) + sqrt(turn_left))
        else:
            output = 0.0

        print "Output (turn degree): %s" % output
            
            
if __name__ == "__main__":
    main()
