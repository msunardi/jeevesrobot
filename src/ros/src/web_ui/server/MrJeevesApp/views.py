from django.shortcuts import render
from django.http import HttpResponse
from django.template import RequestContext
from django.shortcuts import render
from django.core import serializers
from MrJeevesApp.models import ROSTypes,ROSNodes,ROSResults,ROSCommands 
from django.views.decorators.csrf import csrf_exempt
import json
import datetime
import traceback

# Deserializes the JSON data into key-value dictionary
class Payload(object):
     def __init__(self, j):
         self.__dict__ = json.loads(j)
         
def index(request):
 
     battery_status = ROSResults.objects.all().filter(ros_node=1).latest('id')
     add_two_ints = ROSResults.objects.all().filter(ros_node=2).latest('id')
     
     context_dict = {'status' : battery_status, 'add' : add_two_ints}
     
     return render(request, 'MrJeevesApp/index.html', context_dict)

def GetROSNodes(request):
     try:
          context = RequestContext(request)

          data_list =  ROSNodes.objects.all().filter(active=True)

          return  HttpResponse(serializers.serialize('json', data_list), content_type='application/json')
     except:
          return HttpResponse(status=500)


def GetROSResults(request, node_id):
     try:
          context = RequestContext(request)
          
          latest_entry = ROSResults.objects.all().filter(ros_node=node_id).latest('id')
          entries = [latest_entry]
          
          if (entries > 0) :
               result = serializers.serialize('json', entries)
               
               return  HttpResponse(result[1:-1], content_type='application/json')
          
          return HttpResponse(content='node_id ' + node_id + ' cannot be found' , status=500)
     except:
          return HttpResponse(content= "node_id "+ node_id +" cannot be found.", status=500)
     
def GetRosCommands(request, node_id):
     try:
          context = RequestContext(request)
     
          latest_entry = ROSCommands.objects.all().filter(ros_node=node_id, executed = False).latest('id')
          entries = [latest_entry]
          
          if (entries > 0):
               result = serializers.serialize('json', entries)
               
               return  HttpResponse(result[1:-1], content_type='application/json')
          
          return HttpResponse(content='Command for node_id ' + node_id + ' cannot be found' , status=500)
     except:
          return HttpResponse(content='Command for node_id ' + node_id + ' cannot be found', status=500)

@csrf_exempt
def SetCommandExecutedFlag(request, node_id, command_id):
    context = RequestContext(request)
    
    try:
          if request.method == 'POST' :
               #node = ROSNodes.objects.get(pk=node_id)
                    
               ros_command = ROSCommands.objects.get(pk=command_id)
               
               ros_command.executed = True
               
               ros_command.save()
               
               return HttpResponse("OK")
    except:
          traceback.print_exc()
          return HttpResponse(status=500)

@csrf_exempt
def PostROSResults(request):
    context = RequestContext(request)
    #
    #[{"fields":{"result": "{\"voltage\":\"2.5\", \"current\":\"1.0\"}", "ros_node": 1},
    #"model": "MrJeevesApp.rosresults", "pk": 1}]
    #
    
    try:
          #if request.is_ajax():
               if request.method == 'POST':

                    payload_json = request.body
                    
                    ros_result_obj = Payload(payload_json)
                    
                    node = ROSNodes.objects.get(pk=ros_result_obj.fields['ros_node'])
                    
                    ros_result = ROSResults(ros_node = node, created = datetime.datetime.now(), result = json.dumps(ros_result_obj.fields['result']))
                    
                    ros_result.save()
                   
                    return HttpResponse("OK")
               else:
                    return HttpResponse(status=500)
          #else:
          #    return HttpResponse(status=500)
    except:
          traceback.print_exc()
          return HttpResponse(status=500)
    
    


