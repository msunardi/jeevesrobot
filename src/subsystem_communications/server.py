from twisted.internet.protocol import Factory, Protocol
from twisted.internet import reactor
from twisted.python import log

class IphoneChat(Protocol):
	def connectionMade(self):
		self.factory.clients.append(self)
		print "clients are ", self.factory.clients
		print "connection from ", self.transport.getPeer()
		for c in self.factory.clients:
			log.msg("Client connection from %s" % c)

	def connectionLost(self, reason):
		self.factory.clients.remove(self)

	def dataReceived(self, data):
		a = data.split(':')
		print a
		if len(a) > 1:			
			if len(a) == 3:
				sender = a[0]
				target = a[1].rstrip()
				command = a[2].rstrip()
				
				#if target == "body":
				#	msg = target + ":" + command
				#	print msg
				msg = target + ":" + command
				print msg
					
				for c in self.factory.clients:
					c.message(msg)

			else:
				command = a[0]
				content = a[1].rstrip()
				content = content.lstrip()
				print command
				print content
				msg = ""
				if command == "iam":
					self.name = content
					msg = self.name + " has joined."
					print msg
	
				elif command == "msg":
					msg = self.name + ": " + content
					print msg
					#for c in self.factory.clients:
					#	log.msg("Client connection from %s" % c)
				else:
					print "I don't understand."	
	
		else:
			print "I don't understand!"
			msg = "I don't understand what you're trying to do."
		
		for c in self.factory.clients:
			c.message(msg)
			

	def message(self, message):
		self.transport.write(message + '\n')			

factory = Factory()
factory.protocol = IphoneChat
factory.clients = []
reactor.listenTCP(80, factory)
print "Iphone Chat server started"
reactor.run()
