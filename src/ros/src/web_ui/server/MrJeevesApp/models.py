from django.db import models

class ROSTypes(models.Model):
    action_type = models.CharField(max_length=128, unique=True)

    def __unicode__(self):
        return self.action_type
    
class ROSNodes(models.Model):
    ros_type = models.ForeignKey(ROSTypes)
    node_name = models.CharField(max_length=128, unique=True, null=True, blank=True)
    node_topic = models.CharField(max_length=128, unique=True, null=True, blank=True)
    msg_name = models.CharField(max_length=128, unique=True, null=True, blank=True)
    srv_name = models.CharField(max_length=128, unique=True, null=True, blank=True)
    params = models.TextField(null=True, blank=True)
    active = models.BooleanField(default=True)
    
    def __unicode__(self):
        return self.node_name
    
class ROSResults(models.Model):
    ros_node = models.ForeignKey(ROSNodes)
    result = models.TextField(null=True) # Stores Raw JSON Data for ROS to consume
    created = models.DateTimeField(null=True)
    
    def __unicode__(self):
        return self.result
    
class ROSCommands(models.Model):
    ros_node = models.ForeignKey(ROSNodes)
    # Command to send to ROS in JSON 
    command_data = models.TextField(null=True)
    executed = models.BooleanField(default=False)
    
    def __unicode__(self):
        return self.command_data or u''
    

    
    

