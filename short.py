import time
import datetime
import paho.mqtt.client as paho
from paho import mqtt
import base64
from ultralytics import YOLO
import cv2
import math
import sys
import serial
import RPi.GPIO as GPIO
import time

#ser=serial.Serial('/dev/ttyACMO', 9600, timeout=1)
#ser.reset_input_buffer()
# Set up GPIO mode
GPIO.setmode(GPIO.BCM)  # Use Broadcom pin numbering
isConti=False
# Define GPIO pins
pins = [22, 23, 24, 25]

for pin in pins:
    GPIO.setup(pin, GPIO.OUT)
GPIO.output(pins, GPIO.LOW)
# setting callbacks for different events to see if it works, print the message etc.
def on_connect(client, userdata, flags, rc, properties=None):
    print("CONNACK received with code %s." % rc)

# with this callback you can see if your publish was successful
def on_publish(client, userdata, mid, properties=None): 
    print("mid: " + str(mid))

    
client = paho.Client(client_id="Ads1", userdata=None, protocol=paho.MQTTv5)
client.on_connect = on_connect

# enable TLS for secure connection
client.tls_set(tls_version=mqtt.client.ssl.PROTOCOL_TLS)
# set username and password
client.username_pw_set("ads@rpi", "Ads12345678")
# connect to HiveMQ Cloud on port 8883 (default for MQTT)
client.connect("34773dcfbaf24a4bba66e5a333c2df9a.s1.eu.hivemq.cloud", 8883)

# setting callbacks, use separate functions like above for better visibility

client.on_publish = on_publish



mymodel=YOLO('/home/ads/Desktop/rpi/fyp/new/best64auto.pt')
classNames = ["Bear","Boar","Cattle","Deer","Elephant","Horse","Monkey"]

def isval(list1):
    for ele in list1:
        try: 
            print(type(ele))
            print(ele)
            if ele>.7:
                print("IS GREATER")
                return True
        except ValueError:
            print("neds")
    return False

def isAni(list1):
    
    for ele in list1:
        print('animal name:'+ele)
        if ele in classNames:
            return True
    return False

def mqttsender(animallist, sysname, formattedtime):
    animal_counts={}
    for animal in animallist:
        if animal in animal_counts:
            animal_counts[animal]+=1
        else:
            animal_counts[animal]=1
            
    for key, value in animal_counts.items():
        messagetosend=str(classNames.index(key))+"####"+sysname+"####"+str(value)+"####"+formattedtime
        result = client.publish("animal",messagetosend,0)
        print("message sent")
        return
        
def checkingdiffanimalornot(danimallist, animallist):
    letscheck=0
    for animal in animallist:
        for danimal in danimallist:
            if animal == danimal:
                letscheck=letscheck+1
    
    if letscheck == 0:
        return True
    else:
        return False
            
        
        
start_time = time.time()
vid = cv2.VideoCapture(0)
j=0
frameCount=0
found=True
detectedtime=time.time()
detectedanimal=[]
firstprediction=True
while True:
    if frameCount%20!=0:
        frameCount=frameCount+1
        continue 
    ret, frames= vid.read()
    frame=cv2.resize(frames,(320,320))
    result=mymodel(frame, stream=True)
    frameCount=frameCount+1
    
    isValList=[]
    isAniList=[]

    for r in result:
        boxeses=r.boxes
        for box in boxeses:
            x1,y1,x2,y2=box.xyxy[0]
            x1,y1,x2,y2=int(x1),int(y1),int(x2),int(y2)
            org=[x1,y2]
            isValList.append(math.ceil((box.conf[0]*100))/100)
            isAniList.append(classNames[int(box.cls[0])])
            cv2.rectangle(frame, (x1,y1), (x2,y2), (255, 0, 255), 3)
            cv2.putText(frame, classNames[int(box.cls[0])], org, cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 2)
            #cv2.imwrite("/home/ads/Desktop/rpi/fyp/new/pic/{framecount}.jpg", frame);
            
    a=isval(isValList)
    b=isAni(isAniList)
    
    if a and b:
        now=datetime.datetime.now()
        formattedtime=now.strftime("%Y-%m-%d %H:%M:%S")
        print(formattedtime)
        systemname="0001"
        cv2.imwrite(f"pic/{frameCount}.jpg", frame)
        if time.time()-detectedtime>5 or checkingdiffanimalornot(detectedanimal, isAniList):
            detectedanimal=isAniList
            detectedtime=time.time()
            mqttsender(isAniList, systemname, formattedtime)
        if firstprediction:
            mqttsender(isAniList, systemname, formattedtime)
            firstprediction=False

            
            
        GPIO.output(pins[0], GPIO.HIGH)
        GPIO.output(pins[1], GPIO.HIGH)
        GPIO.output(pins[2], GPIO.HIGH)
        GPIO.output(pins[3], GPIO.HIGH)
    else:
        GPIO.output(pins[0], GPIO.LOW)
        GPIO.output(pins[1], GPIO.LOW)
        GPIO.output(pins[2], GPIO.LOW)
        GPIO.output(pins[3], GPIO.LOW)
        
                
                          
client.loop_forever()   
vid.release()
cv2.destroyAllWindows()
GPIO.cleanup()

#


