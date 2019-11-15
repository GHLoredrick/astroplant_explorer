#Script om sensoren uit te lezen. Dit script gebruikt aangepaste versies van script block2 met 1 waarde.
#To do: header, elke dag een nieuwe file incl datum
from time import strftime,sleep
import os # os is deprecated and to be replaced by subprocess
import csv
#for i in range(1,2):
temp = os.popen('python3 waterTemp_1x.py').readline()#!!! if you want to run the script with crontab specify the path !!!
co2 = os.popen('python3 co2_1x.py').readline()
light = os.popen('python3 light_1x.py').readline()
hum = os.popen('python3 DHT_1x.py').readline()#gebruik alleen humidity van DHT sensor
bestand = open("/home/pi/Desktop/test5.csv", "a")#"w" create new file, "a" adds data
with bestand:
  writer = csv.writer(bestand, delimiter=",")
  writer.writerow([strftime("%Y-%m-%d %H:%M"),str(temp),str(co2), str(light),str(hum)])
   
    

