import os
import django

def populate():
    t = ROSActionType.objects.get_or_create(action_type='foo')[0]
    
    r = add_ROSActionRegister(actiontype=t,
                        name='Battery_Status',
                        servername='servername',
                        clientname="clientname")
    
    add_ROSActionOutput(r, {'data':"foo"})
    
    
def add_ROSActionRegister(actiontype, name , servername = None, clientname = None, publishername=None, subscribername=None):
    p = ROSActionRegister.objects.get_or_create(action_type=actiontype,
                                                name=name,
                                                server_name=servername,
                                                client_name=clientname,
                                                publisher_name=publishername,
                                                subscriber_name=subscribername)[0]
    return p


def add_ROSActionOutput(action, output=None):
    o = ROSActionOutput.objects.get_or_create(ros_action = action, output = data)[0]
    return o


# Start execution here!
if __name__ == '__main__':
    
    print "Starting MrJeevesApp population script..."
    os.environ.setdefault('DJANGO_SETTINGS_MODULE', 'MrJeeves_Project.settings')
    from MrJeevesApp.models import ROSActionType, ROSActionRegister, ROSActionOutput
    django.setup()
    populate()