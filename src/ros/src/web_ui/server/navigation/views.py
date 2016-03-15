from django.shortcuts import render, redirect
from django.http import HttpResponse
from django.views.generic import View
from django import forms

from Navigator import WaypointManager
# Create your views here.
#def index(request):  
#    return HttpResponse('Navigation')

class WaypointView(View):
    def __init__(self):
        super(WaypointView, self).__init__()
        self.nav = WaypointManager()
    
    def get(self, request):
        try:            
            waypoints = self.nav.get_waypoints()
            qs = request.GET
            context_dict = {'waypoints': waypoints, 'qs': qs}
            #return HttpResponse("Navigation View %s" % waypoints)
            return render(request, 'navigation/index.html', context_dict)
        except Exception as e:
            return HttpResponse("Foobar! %s" % e)

class SaveCurrentPoseView(View):
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
