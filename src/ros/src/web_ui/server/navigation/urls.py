from django.conf.urls import patterns, url
from navigation.views import WaypointView, SaveCurrentPoseView

urlpatterns = [
    url(r'^$', WaypointView.as_view(), name='index'),
    url(r'^save_current_pose', SaveCurrentPoseView.as_view(), name='save_current_pose'),
]
