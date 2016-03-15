from django.shortcuts import render
from django.http import HttpResponse
from django.views.generic import View

from Navigator import WaypointManager
# Create your views here.
#def index(request):  
#    return HttpResponse('Navigation')

class NavigationView(View):
    def __init__(self):
        super(NavigationView, self).__init__()
        self.nav = WaypointManager()
    
    def get(self, request):
        try:
            waypoints = self.nav.get_waypoints()
            context_dict = {'waypoints': waypoints}
            #return HttpResponse("Navigation View %s" % waypoints)
            return render(request, 'navigation/index.html', context_dict)
        except:
            return HttpResponse("Foobar!")
