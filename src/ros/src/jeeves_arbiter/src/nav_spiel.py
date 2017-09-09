#!/usr/bin/env python

import sys, os, time, json, string
import threading

import rospy
import aiml                               # Artificial Intelligence Markup Language
from std_msgs.msg import String
import random as r
import time
import subprocess
import redis
from actionlib_msgs.msg import GoalStatusArray

class NavSpiel(threading.Thread):

    def __init__(self):
        # Set current working directory
        cwd = os.chdir(os.path.dirname(os.path.realpath(__file__)) + '/../');
        cwd = os.getcwd() + '/';

        self.jeeves = aiml.Kernel()
        self.jeeves.learn(cwd + "jeeves.xml")
        self.jeeves.respond('load aiml b') #finished initializing

        self.r = redis.Redis()

        self.locations = {'ee': 'EE FISHBOWL', \
                    'NEAR_lab': 'NEAR LAB', \
                    'near_lab': 'NEAR LAB', \
                    'robot_lab': 'ROBOTICS LAB', \
                    'Biomedical_lab': 'BIOMEDICAL SIGNAL PROCESSING LAB', \
                    'biomedical_lab': 'BIOMEDICAL SIGNAL PROCESSING LAB', \
                    'stairs': 'EB STAIRS', \
                    'portland_state': 'PORTLAND STATE', \
                    'portland_state_university': 'PORTLAND STATE'}

        self.facts_or_jokes = ['FACTS', 'JOKES']
        self.comment = {'FACTS': ['By the way, did you know . . .', 'It reminds me . . .', 'Here is a little trivia . . .'],
                'JOKES': ['Funny story ...', 'This is a true story ...'],
                'CLOSING': {'FACTS': ['. . . Interesting, isn\'t it?', '. . . Fascinating', '. . . Remarkable', 'Fascinating, no?', ''],
                            'JOKES': ['. . Ha ha ha', 'Get it? . . Heh Heh.', '. . . Heh heh heh.', ''],
                            'END': ['Thank you for your attention.', 'I hope you enjoyed it.', 'Let me know if there is anything else I can do for you.']}}

        self.sub = rospy.Subscriber('chatter', String, queue_size=10)
        self.say = rospy.Publisher('/jeeves_speech/speech_synthesis', String, queue_size=10)

        self.spiel = True
        self.arrived = False
        self.speaking = False
        self.stop_all = False

        self.collect = 4
        self.place = ''

        rospy.Subscriber("/spiel", String, self.callback)
        rospy.Subscriber("/done", String, self.done)
        rospy.Subscriber("/goal_reached", String, self.goal_reached)
        rospy.Subscriber("/move_base/status", GoalStatusArray, self.goal_reached_action)

        threading.Thread.__init__(self)
        self.sleeper = rospy.Rate(10)

    def callback(self, message):

        # topic = "PORTLAND STATE"
        # topic = "INTEL LAB"
        # topic = self.locations[message.data]
        itype, loc = message.data.split(':')
        topic = self.locations[loc]
        self.place = topic
        self.arrived = False
        rcvd = []   # Container for spiel
        self.r.set('current_target', topic)

        # Probability of inserting a joke/fact
        jk_prob = 0.05
        jk_prob = 0.2
        
        fj_count = 0
        for i in range(self.collect):
            rsp = self.jeeves.respond(topic)

            # If don't know, abort
            if not rsp:
                rcvd.append('I am sorry, but I do not know much about %s yet. ' % topic)
                break

            # Collect info
            if rsp not in rcvd:
                rcvd.append(rsp)
                print rsp

            # Insert random joke/fact
            ran = r.random()
            if ran < jk_prob:
                factjoke = r.choice(self.facts_or_jokes)
                rx = self.jeeves.respond(factjoke)
                if rx not in rcvd:
                    rx = r.choice(self.comment[factjoke]) + '. ' + rx + '. ' + r.choice(self.comment['CLOSING'][factjoke])
                    rcvd.append(rx)
                    print rx
        rcvd.append('%s that is all I can say about %s %s. %s' % (r.choice(['And', 'Well, ', '']), topic, r.choice(['for now', '']), r.choice(self.comment['CLOSING']['END'])))

        for msg in rcvd:
            if not self.spiel:
                self.spiel = True
                msg = "OK. I'll stop."
                self.festival(msg)
                break
            try:
                while self.speaking:
                    pass
                self.festival(msg)
            except Exception as e:
                print e
            finally:
                sl = r.random()*0.5
                print "I'm done - sleeping for %s" % sl
                time.sleep(sl)
        else:
            self.spiel = False
        print "After spiel - Arrived: %s" % self.arrived

        if itype != 'nav':
            return
        
        while not self.arrived and not self.spiel:
            try:
                if self.arrived or self.stop_all:
                    self.spiel = True
                    msg = "OK. I'll stop."
                    self.festival(msg)
                    self.stop_all = False
                    break
                factjoke = r.choice(self.facts_or_jokes)
                rx = self.jeeves.respond(factjoke)
                rx = r.choice(self.comment[factjoke]) + '. ' + rx + '. ' + r.choice(self.comment['CLOSING'][factjoke])

                while self.speaking and not self.arrived:
                    pass
                if self.arrived and r.random() < 0.3:
                    break
                self.festival(rx)
            except Exception as e:
                print e
            finally:
                sl = r.random()*3.0
                print "Not arrived yet - sleeping for %s" % sl
                time.sleep(sl)
        self.arrived = False
        self.spiel = True
        self.stop_all = False

    def done(self, message):
        if message.data == "no":
            self.spiel = True
            self.stop_all = False
        else:
            self.spiel = False
            self.stop_all = True

        self.arrived = False

    def goal_reached(self, message):
        print "Arrived: %s" % self.arrived
        if 'Goal reached' in message.data and not self.arrived:
            arrive = ['We have reached %s.', 'Well, here we are . . . %s', 'Welcome to the %s . . . ', 'This is the %s . . . ']
            #msg = r.choice(arrive) % self.place
            msg = r.choice(arrive) % self.r.get('current_target')
            if self.speaking or self.spiel:
                return
            self.festival(msg)
            self.arrived = True
            print "Yes Arrived: %s" % self.arrived

    def goal_reached_action(self, message):
        print "True Arrived: %s - %s" % (self.arrived, self.place)
        try:
            if 'Goal reached' in message.status_list[-1].text and not self.arrived:
                arrive = ['We have reached %s.', 'Well, here we are . . . %s', 'Welcome to the %s . . . ', 'This is the %s . . . ']
                #msg = r.choice(arrive) % self.place
                msg = r.choice(arrive) % self.r.get('current_target')
                if self.speaking or self.spiel:
                    return
                self.festival(msg)
                self.arrived = True
                print "Yes Arrived: %s" % self.arrived
        except IndexError as e:
            warn = "Arbiter.nav_spiel.py: Not receiving GoalStatusArray.status_list yet."
            rospy.logwarn(warn)
            print warn

    def festival(self, say):
        self.speaking = True
        try:
            subprocess.call(["festival", "--batch", "(SayText \"" + say + "\")"])
        except Exception as e:
            rospy.logerr('Festival subprocess call failed.')
        finally:
            self.speaking = False

    def run(self):
        while not rospy.is_shutdown():
            self.sleeper.sleep()

    # def nav_spiel():
    #     rospy.init_node('nav_spiel', anonymous=True)
    #     rospy.Subscriber("/spiel", String, callback)
    #     rospy.Subscriber("/done", String, done)
    #     rospy.Subscriber("/goal_reached", String, goal_reached)
    #     rospy.Subscriber("/move_base/status", GoalStatusArray, goal_reached_action)
    #     # spin() simply keeps python from exiting until this node is stopped
    #     rospy.spin()

def main(args):
    rospy.init_node('nav_spiel', anonymous=True)
    ns = NavSpiel()
    ns.start()
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
