from twisted.internet.protocol import ClientFactory
from twisted.protocols.basic import LineReceiver
from twisted.internet import reactor
import sys

from os import system
import subprocess
import time

class SpeakClient(LineReceiver):

    def connectionMade(self):
        print "Connected to server."
        self.sendLine("iam:speaker\n")

    def dataReceived(self, line):
        print "received: ", line

        message = line.split(':')

        if len(message) > 1:
            if message[1] == "speaker":
                #speak = "espeak \"" + message[2] + "\""
                #speak = message[2] + "\|espeak"
                #if system(speak) == 0:
                #    print "done!\n"
                print message[2]
                code = subprocess.call(["espeak", message[2]])
                time.sleep(0.3)
                

            #else:
            #    print line

class SpeakClientFactory(ClientFactory):
    protocol = SpeakClient

    def clientConnectionFailed(self, connector, reason):
        print 'connection failed: ', reason.getErrorMessage()
        reactor.stop()

    def clientConnectionLost(self, connector, reason):
        print 'connection lost: ', reason.getErrorMessage()
        reactor.stop()


factory = SpeakClientFactory()
reactor.connectTCP('localhost', 8008, factory)
reactor.run()
