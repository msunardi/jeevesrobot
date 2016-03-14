from django.shortcuts import render
from django.http import HttpResponse
from django.template import RequestContext
from django.shortcuts import render
from django.core import serializers
from MrJeevesApp.models import QR
#from django.views.decorators.csrf import csrf_exempt
import json
import traceback
from ROSClients import ROSClients
import ROSSubscribers

import logging
logger = logging.getLogger(__name__)

# Deserializes the JSON data into key-value dictionary
class Payload(object):
     def __init__(self, j):
         self.__dict__ = json.loads(j)
         
def index(request):

     client = ROSClients()
     # convert to json payload
     resp_json = Payload(client.get_qrcode());
     
     context_dict = {}
     if resp_json:
         context_dict = {"Id" : resp_json.id}
     
     return render(request, 'MrJeevesApp/index.html', context_dict)

    

# QR Code Controller 
def FindAndMatchQR(request):
     
     try:
          client = ROSClients()
          # convert to json payload
          pos = Payload(client.get_qrcode());
          
          # default message if not found
          message = 'Not Found'
          
          if(not pos.valid):
               message = 'Not Valid'
          else:
               message = "id: %d, r: %f, theta:%f" % (pos.id, pos.r, pos.theta)
          #else:
               # check if the id matches and get the first data when matched
          #     result = QR.objects.filter(QR_ID = resp_json.id).first()
               
               # get the message if there is a match
          #     if result is not None:
          #          message = result.Message
          logger.info("Got message: %s" % message) 
          #return  HttpResponse(serializers.serialize('json', message), content_type='application/json')
          return HttpResponse('We got: %s' % message)

     except Exception as e:
          traceback.print_exc()
          logger.error("Uh-oh %s" % e)
          return HttpResponse(status=500)
