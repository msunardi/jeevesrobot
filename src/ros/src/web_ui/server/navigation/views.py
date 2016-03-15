from django.shortcuts import render, redirect
from django.http import HttpResponse
from django.views.generic import View
from django import forms

from Navigator import WaypointManager
# Create your views here.
#def index(request):  
#    return HttpResponse('Navigation')

nav = WaypointManager()

class WaypointView(View):
    # Main navigation view
    
    def get(self, request, *args, **kwargs):
        try:            
            # Dictionary of known waypoints
            waypoints = nav.get_waypoints()

            # qs checks for querystring e.g. successfully adding new waypoint
            qs = request.GET
            context_dict = {'waypoints': waypoints, 'qs': qs}
            if 'message' in kwargs:
                context_dict['message'] = kwargs['message']
            #return HttpResponse("Navigation View %s" % qs)
            return render(request, 'navigation/index.html', context_dict)
        except Exception as e:
            return HttpResponse("Foobar! %s" % e)

class SaveCurrentPoseView(View):
    # View/Endpoint to add new waypoint
    
    def post(self, request, *args, **kwargs):
        try:
            msg = ""
            waypoint_name = request.POST.get('waypoint_name', "")
            if waypoint_name: 
                response = nav.save_current_pose(waypoint_name)
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

    def get(self, request, *args, **kwargs):
        try:
            wp = request.GET.get('waypoint') 
            response = nav.delete_waypoint(wp)
            return redirect('/navigation?d=0')
        except Exception as e:
            return HttpResponse(e)

class GoToWaypointView(View):
    def get(self, request, *args, **kwargs):
        try:
            wp = request.GET.get('waypoint')
            response = nav.goto_waypoint(wp)
            oof = ""
            for k, v in response.items():
                oof += "%s: %s\n" % (k, v)
            return HttpResponse(oof)
        except Exception as e:
            return HttpResponse(e)

class CancelWaypointView(View):
    def get(self, request):
        try:
            response = nav.cancel_current_goal()
            return redirect('/navigation?c=0')
        except Exception as e:
            return HttpResponse(e)
