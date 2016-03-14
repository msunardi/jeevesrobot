from django.db import models


class QR(models.Model):
    QR_ID = models.IntegerField(unique=True, null=False, blank=False)
    Message = models.TextField(null=False, default='new message', blank=False)
    
    def __unicode__(self):
        return self.QR_ID

    
