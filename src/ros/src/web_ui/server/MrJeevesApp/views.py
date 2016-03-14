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

# Deserializes the JSON data into key-value dictionary
class Payload(object):
     def __init__(self, j):
         self.__dict__ = json.loads(j)
         
def index(request):

     client = ROSClients()
     # convert to json payload
     resp_json = Payload(client.QRClient());
     
     context_dict = {"Id" : resp_json.id}
     
     return render(request, 'MrJeevesApp/index.html', context_dict)

# QR Code Controller 
def FindAndMatchQR(request):
     
     try:
          client = ROSClients()
          # convert to json payload
          resp_json = Payload(client.QRClient());
          
          # default message if not found
          message = 'Not Found'
          
          if(not resp_json.valid):
               message = 'Not Valid'
          else:
               # check if the id matches and get the first data when matched
               result = QR.objects.filter(QR_ID = resp_json.id).first()
               
               # get the message if there is a match
               if result is not None:
                    message = result.Message
          
          return  HttpResponse(serializers.serialize('json', message), content_type='application/json')
     except Exception as e:
          traceback.print_exc()
          return HttpResponse(status=500)
