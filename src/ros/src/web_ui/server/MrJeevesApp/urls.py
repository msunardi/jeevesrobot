from django.conf.urls import patterns, url
from MrJeevesApp import views

urlpatterns =   [
                url(r'^$', views.index, name='index'),
                url(r'^FindAndMatchQR', views.FindAndMatchQR, name='FindAndMatchQR')
                ]