from django.conf.urls import patterns, url
from navigation.views import NavigationView

urlpatterns = [
    url(r'^$', NavigationView.as_view(), name='index'),
]
