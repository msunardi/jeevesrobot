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
		json = False
		a = data.split('!')
		print a[0]
		if a[0]=="json":
			print "json true"
			json = True
			msg = a[1:]
			print "json: "
			#print msg[0]
		else:
			a = data.split(':')
			print "data: "
			print a
			if (len(a) > 1) and not json:			
				if len(a) == 3:
					sender = a[0]
					target = a[1].rstrip()
					command = a[2].rstrip()
					
					#if target == "body":
						#	msg = target + ":" + command
						#	print msg
					#msg = target + ":" + command
					msg = sender + ":" + target + ":" + command
					print msg
					
					#for c in self.factory.clients:
					#	c.message(msg)

				elif len(a) == 2:
					command = a[0]
					content = a[1].rstrip()
					content = content.lstrip()
					print command
					print content
					msg = ""
					if command == "iam":
						self.name = content
						#msg = self.name + " has joined." # Don't need to print this as msg (broadcasted)
						# It will make clients receive this comment along with the actual message
						print self.name + " has joined."
						#msg = "iam:" + self.name
	
					elif command == "msg":
						msg = self.name + ": " + content
						print msg
						#for c in self.factory.clients:
						#	log.msg("Client connection from %s" % c)
					else:
						print "I don't understand."	
                        	else:
                        		msg = data
	
			else:
				print "I don't understand!"
				msg=""
				#msg = "I don't understand what you're trying to do."
		
		print "messaging...\n"	
		for c in self.factory.clients:
			if not json:
				c.message(msg)
			else:
				c.message(msg[0])
			

	def message(self, message):
		self.transport.write(message + '\n')			

factory = Factory()
factory.protocol = IphoneChat
factory.clients = []
reactor.listenTCP(8008, factory)
print "Jeeves' server started"
reactor.run()
