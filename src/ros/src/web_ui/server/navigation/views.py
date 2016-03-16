from django.shortcuts import render, redirect
from django.http import HttpResponse, JsonResponse
from django.views.generic import View
from django import forms

from Navigator import WaypointManager, NavTest
# Create your views here.
#def index(request):  
#    return HttpResponse('Navigation')

SUCCESS = 0
FAIL = -1

nav = WaypointManager()
navtest = NavTest()


class WaypointView(View):
    # Main navigation view
    
    def get(self, request, *args, **kwargs):
        try:            
            # Dictionary of known waypoints
            waypoints = nav.get_waypoints()

            # qs checks for querystring e.g. successfully adding new waypoint
            qs = request.GET
            context_dict = {'waypoints': waypoints, 'qs': qs}
            if 'message' in self.kwargs:
                context_dict['message'] = self.kwargs['message']
                return HttpResponse("Navigation View %s" % context_dict)
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
                    msg = SUCCESS
                else:
                    msg = FAIL
                return redirect('/navigation?s=%s'% msg)
            else:
                return redirect('/navigation?s='+FAIL)
        except:
            return HttpResponse("[SaveCurrentPoseView] Something went wrong...")

class DeleteWaypointView(View):
    # Delete a waypoint

    def get(self, request, *args, **kwargs):
        try:
            wp = request.GET.get('waypoint') 
            response = nav.delete_waypoint(wp)
            return redirect('/navigation?d='+SUCCESS)
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
            return redirect('/navigation', kwargs={'success': True})
        except Exception as e:
            return HttpResponse(e)

class SetCurrentPoseWaypointView(View):
    def get(self, request):
        try:
            wp = request.GET.get('waypoint')
            response = nav.set_current_pose_to_waypoint(wp)
            return redirect('/navigation?w='+SUCCESS)
        except Exception as e:
            return redirect('/navigation?w='+FAIL)
            #return HttpResponse(e)

class NavTestView(View):
    def get(self, request, *args, **kwargs):
        try:
            cmd = self.kwargs['cmd']
            #response = navtest.cmd_nav_test(cmd)
            if cmd == "RUN":
                msg = "Nav Test is running."
            elif cmd == "HALT":
                msg = "Nav Test has stopped."
            else:
                msg = "I don't know what's going on here ..."
            return HttpResponse(msg)
        except Exception as e:
            return HttpResponse("Fubar! %s" % e) 

class NavTestStatusView(View):
    def get(self, request, *args, **kwargs):
        try:
            status = navtest.cmd
            last_message = navtest.last_message
            progress = navtest.progress
            full_status = {'status': status, 'last_message': last_message, 'progress': progress}
            return JsonResponse(full_status)

        except Exception as e:
            return HttpResponse(e)
