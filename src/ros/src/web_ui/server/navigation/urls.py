from django.conf.urls import patterns, url
from navigation.views import WaypointView, SaveCurrentPoseView,\
    DeleteWaypointView, GoToWaypointView, \
    CancelWaypointView, SetCurrentPoseWaypointView, \
    NavTestView, NavTestStatusView

urlpatterns = [
    url(r'^$', WaypointView.as_view(), name='navigation'),
    url(r'^save_current_pose', SaveCurrentPoseView.as_view(), name='save_current_pose'),
    url(r'^delete_waypoint', DeleteWaypointView.as_view(), name='delete_waypoint'),
    url(r'^goto_waypoint', GoToWaypointView.as_view(), name='goto_waypoint'),
    url(r'^cancel_waypoint', CancelWaypointView.as_view(), name='cancel_waypoint'),
    url(r'^set_current_waypoint', SetCurrentPoseWaypointView.as_view(), name='set_current_waypoint'),
    url(r'^navtest/(?P<cmd>\w+)', NavTestView.as_view(), name='nav_test'),
    url(r'^navteststatus', NavTestStatusView.as_view(), name='nav_test_status'),
]
