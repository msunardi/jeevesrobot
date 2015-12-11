from django.conf.urls import patterns, url
from MrJeevesApp import views

urlpatterns = patterns('',
        url(r'^$', views.index, name='index'),
        url(r'^GetROSNodes/', views.GetROSNodes, name='GetROSNodes'),
        url(r'^GetROSResults/(?P<node_id>\w+)/$', views.GetROSResults, name='GetROSResults'),
        url(r'^PostROSResults/', views.PostROSResults, name='PostROSResults'),
        url(r'^GetRosCommands/(?P<node_id>\w+)/$', views.GetRosCommands, name='GetRosCommands'),
        url(r'^SetCommandExecutedFlag/(?P<node_id>\w+)/(?P<command_id>\w+)$', views.SetCommandExecutedFlag, name='SetCommandExecutedFlag'),)