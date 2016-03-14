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
        return HttpResponse("Navigation View")
