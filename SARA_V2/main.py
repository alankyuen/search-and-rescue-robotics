from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
from pymavlink import mavutil  # Needed for command message definitions
import SARA

#instantiate SARA object
rover = SARA.SARA()

#initialize SARA
rover.init()
while(rover.MISSION_ENABLED):
    rover.run()
rover.deconstruct()



"""

steve’s phone: 192.168.43.62
ben's phone: 172.20.10.2
alan's phone: 172.20.10.6

scp /Users/alanyuen/Desktop/RAG-2017-Info-Processing-master/SARA_V2/SARA.py odroid@192.168.43.62:~/Documents/

scp /Users/alanyuen/Desktop/RAG-2017-Info-Processing-master/SARA_V2/main.py odroid@192.168.43.62:~/Documents/

scp /Users/alanyuen/Desktop/RAG-2017-Info-Processing-master/SARA_V2/field.py odroid@192.168.43.62:~/Documents/

scp /Users/alanyuen/Desktop/RAG-2017-Info-Processing-master/SARA_V2/cluster.py odroid@192.168.43.62:~/Documents/

scp /Users/alanyuen/Desktop/RAG-2017-Info-Processing-master/SARA_V2/dbscan.py odroid@192.168.43.62:~/Documents/

scp /Users/alanyuen/Desktop/RAG-2017-Info-Processing-master/SARA_V2/constants.py odroid@192.168.43.62:~/Documents/


"""
