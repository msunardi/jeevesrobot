from django.shortcuts import render, redirect
from django.http import HttpResponse
from django.views.generic import View
from django import forms

from Navigator import WaypointManager
# Create your views here.
#def index(request):  
#    return HttpResponse('Navigation')

class WaypointView(View):
    # Main navigation view
    def __init__(self):
        super(WaypointView, self).__init__()
        self.nav = WaypointManager()
    
    def get(self, request, *args, **kwargs):
        try:            
            # Dictionary of known waypoints
            waypoints = self.nav.get_waypoints()

            # qs checks for querystring e.g. successfully adding new waypoint
            qs = request.GET
            context_dict = {'waypoints': waypoints, 'qs': qs}
            #return HttpResponse("Navigation View %s" % qs)
            return render(request, 'navigation/index.html', context_dict)
        except Exception as e:
            return HttpResponse("Foobar! %s" % e)

class SaveCurrentPoseView(View):
    # View/Endpoint to add new waypoint
    def __init__(self):
        super(SaveCurrentPoseView, self).__init__()
        self.nav = WaypointManager()
    
    def post(self, request, *args, **kwargs):
        try:
            msg = ""
            waypoint_name = request.POST.get('waypoint_name', "")
            if waypoint_name: 
                response = self.nav.save_current_pose(waypoint_name)
                if not response:
                    msg = 0
                else:
                    msg = response
                return redirect('/navigation?s=%s'% msg)
            else:
                return redirect('/navigation')
        except:
            return HttpResponse("[SaveCurrentPoseView] Something went wrong...")

class DeleteWaypointView(View):
    # Delete a waypoint
    def __init__(self):
        super(DeleteWaypointView, self).__init__()
        self.nav = WaypointManager()

    def get(self, request, *args, **kwargs):
        try:
            #return HttpResponse("Hello world! %s" % request.GET.get('waypoint'))
            response = self.nav.delete_waypoint(request.GET.get('waypoint'))
            return redirect('/navigation?d=0')
        except Exception as e:
            return HttpResponse(e)
