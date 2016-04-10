#!/usr/bin/env python

import wikipedia

from wiki.srv import *
import rospy

def handle_wiki_request(req):
	print "Requested entry: %s" % (req.entry)
	blurb = wikipedia.summary(req.entry, sentences=2)
	return WikiEntryResponse(blurb)

def wiki_entry_server():
	rospy.init_node('wiki_entry_server')
	s = rospy.Service('wiki_entry', WikiEntry, handle_wiki_request)
	print "Ready to ask Wikipedia..."
	rospy.spin()

if __name__ == "__main__":
	wiki_entry_server()