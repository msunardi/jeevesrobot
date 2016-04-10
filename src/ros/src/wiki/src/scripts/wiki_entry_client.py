#!/usr/bin/env python

import sys
import rospy
from wiki.srv import *

def wiki_entry_client(entry):
	rospy.wait_for_service('wiki_entry')
	try:
		wiki_entry_service = rospy.ServiceProxy('wiki_entry', WikiEntry)
		response = wiki_entry_service(entry)
		return response.summary
	except rospy.ServiceException, e:
		print "Service call failed: %s" % e

def usage():
	return "Foo!"

if __name__ == "__main__":
	if str(sys.argv[1]):
		entry = str(sys.argv[1])
	else:
		print usage()
		sys.exit(1)
	print "Requesting %s" % entry
	print "Summary for %s: %s" % (entry, wiki_entry_client(entry))