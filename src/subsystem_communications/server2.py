# Reference: http://twistedmatrix.com/documents/current/core/howto/servers.html
from twisted.internet.protocol import Factory, Protocol
from twisted.protocols.basic import LineReceiver
from twisted.internet import reactor
from twisted.python import log

class MessageListener(LineReceiver):
	# The Protocol sub-class that handles client activities (e.g. connecting, messaging, etc)
	# It's basically a chat server ...

	# class initialization/constructor
	def __init__(self, components):
		self.components = components 
		self.name = None
		self.state = "INIT"

	# Overriding/defining the Protocol object connectionMade() method
	# invoked when client first connects to server (one-time) event
	def connectionMade(self):
		print "Connection from: ", self.transport.getPeer()
		self.sendLine("Id please?\n")

	# invoked when client disconnects from server
	def connectionLost(self, reason):
		if self.components.has_key(self.name):
			del self.components[self.name]

	# invoked when there is a new message coming
	def dataReceived(self, line):
		try:
			msg = line.split(':')
			if self.state == "INIT":
				if msg[0] == "iam":
					self.callback_NEWCOMPONENT(msg[1])
				else:
					self.sendLine("Please identify yourself to proceed. Message me: \"iam:your_name\"\n")
					return
			else:
				self.callback_BROADCAST(line)
		except:
			self.sendLine("Unknown message format\n")

	# method when a new component registers itself
	def callback_NEWCOMPONENT(self, name):
		# Check if component already existed
		if self.components.has_key(name):
			self.sendLine("Name already taken. Try something else.\n")
			return
		self.sendLine("%s connected!" % (name,))
		self.name = name
		self.components[name] = self
		self.state = "CONN"
		#print "%s is connected\n" % name

	# method when components (i.e. client) sends commands
	def callback_BROADCAST(self, message):
		m = message.split(':')
		sender = m[0]
		target = m[1].rstrip()
		command = m[2:]

		if target == "all":
			for name, protocol in self.components.iteritems():
				protocol.sendLine(':'.join(command)+"\n")
		else:
			for name, protocol in self.components.iteritems():
				# Only send command to specific targets
				if (protocol != self) and (name == target):
					protocol.sendLine(':'.join(command)+"\n") 

class ChatFactory(Factory):

	def __init__(self):
		print "Intializing server..."
		self.components = {}

	def buildProtocol(self, addr):
		return MessageListener(self.components)

reactor.listenTCP(8008, ChatFactory())
print "Server started"
reactor.run()

