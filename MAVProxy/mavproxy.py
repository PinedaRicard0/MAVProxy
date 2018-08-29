#Saved by Virtuxplorer
#Saved by Virtuxplorer
#Saved by Virtuxplorer
#!/usr/bin/env python


'''
mavproxy - a MAVLink proxy program

Copyright Andrew Tridgell 2011
Released under the GNU GPL version 3 or later

'''
'VIRTUXPLORER MAVPROXY 2018'

import subprocess
import re
import math
from math import floor
import sys, os, time, socket, signal
import fnmatch, errno, threading
import serial, Queue, select
import traceback
import select
import shlex
import socket
import time, math
import threading
import requests
import json
import urllib
import urllib2
import netifaces as ni
import math
from flask import Flask
from threading import Timer
from flask import request
from pymavlink import mavutil
from threading import Thread
from MAVProxy.modules.lib import textconsole
from MAVProxy.modules.lib import rline
from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import dumpstacks
from MAVProxy.modules.lib import mp_module
import datetime


currentVersionMavproxy='vx.v.4'
flagSetChronoTime=False
timeFlight=0
objConnection=''
objSupervisor=''
objFence=''
objToken=''
app = Flask(__name__)


#La siguiente clase permite realizar acciones como despegar, realizar misiones(doMission), regresar a casa, cambiar a modo de vuelo manual(POSHOLD)
class FlightAction(object):

	def __init__(self):
		self.flagTakeoff=False

	def getFlagTakeoff(self):
		return self.flagTakeoff

	def setFlagTakeoff(self,aux):
		self.flagTakeoff=aux

	#La siguiente funcion se encarga de despegar el dron 

	def takeoff(self):


		global objFence
		global objSupervisor
		
		#Con las siguientes 2 variables obtenemos el estado de armado y el modo de vuelo
		stateArmed=mpstate.master().motors_armed()
		flightMode=str(mpstate.status.flightmode)
		
		#Primero validamos el estado de armado del dron, si es igual a falso, activamos los motores con el comando arm throttle.

		if(stateArmed==False or self.flagTakeoff==False):

			if(stateArmed==False):

				self.flagTakeoff=False
				
				#El canal 3 lo activamos en 1500(valor estable)
				line='rc 3 1500'
				mpstate.input_queue.put(line)
				line='rc 1 0'
				mpstate.input_queue.put(line)
				line='rc 2 0'
				mpstate.input_queue.put(line)
				line='rc 4 0'
				mpstate.input_queue.put(line)
				line='GUIDED'
				mpstate.input_queue.put(line)
				#El siguiente comando permite activar los motores
				line='arm throttle'
				mpstate.input_queue.put(line)
				save_log("MESSAGE: Arming drone!")

			#Cuando el modo de vuelo sea Guided y el estado de armado sea igual a true, entonces despegamos el dron

			elif(flightMode=="GUIDED" and stateArmed==True and self.flagTakeoff==False):

				print "@@@@@@@@@@@@@@@DESPEGANDO@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@"
				#Capturamos el valor de la altura minima de la zona
				heightMin=objFence.getHeightMin()
				attitude=heightMin+1
				#Con el siguiente comando despegamos el dron a la altura minima de la zona
				line='takeoff '+ str(attitude)
				mpstate.input_queue.put(line)
				self.flagTakeoff=True
				save_log("MESSAGE: Takeoff!")
				#Si la siguiente validacion es igual a Falso, se activa el hilo checkStateMultirotor perteneciente a la clase Supervisor
				if(objSupervisor.getFlagCheckState()==False):
					objSupervisor.setFlagCheckState(True)
					Timer(1.0, objSupervisor.checkStateMultirotor).start()

			Timer(2.0,self.takeoff).start()

	#El siguiente metodo sirve para realizar una mision de forma automatica

	def doMission(self):

		stateArmed=mpstate.master().motors_armed()
		flightMode=str(mpstate.status.flightmode)

		#Se realiza la siguiente validacion para verificar que el dron no encuentre en modo Auto y los motores activados

		if(flightMode!="AUTO" or stateArmed==False):

			if(stateArmed==False):
				
				#Con los siguientes comandos activamos primero los motores cuando esta desactivado
				self.flagTakeoff=False
				line='rc 3 1500'
				mpstate.input_queue.put(line)
				line='rc 1 0'
				mpstate.input_queue.put(line)
				line='rc 2 0'
				mpstate.input_queue.put(line)
				line='rc 4 0'
				mpstate.input_queue.put(line)
				line='GUIDED'
				mpstate.input_queue.put(line)
				line='arm throttle'
				mpstate.input_queue.put(line)
				save_log("MESSAGE: Arming drone!")


			#Si se cumple la siguiente condicion, el dron se coloca en modo de vuelo automatico
			
			elif(flightMode!="AUTO" and stateArmed==True):

				self.flagTakeoff==True      
				line='rc 1 1500'
				mpstate.input_queue.put(line)
				line='rc 2 1500'
				mpstate.input_queue.put(line)
				line='rc 3 1500'
				mpstate.input_queue.put(line)
				line='rc 4 1500'
				mpstate.input_queue.put(line)
				line='rc 1 0'
				mpstate.input_queue.put(line)
				line='rc 2 0'
				mpstate.input_queue.put(line)
				line='rc 3 0'
				mpstate.input_queue.put(line)
				line='rc 4 0'
				mpstate.input_queue.put(line)
				line='AUTO'
				mpstate.input_queue.put(line)
				save_log("MESSAGE: Mode auto activated")
				#Si la siguiente validacion es igual a Falso, se activa el hilo checkStateMultirotor perteneciente a la clase Supervisor
				if(objSupervisor.getFlagCheckState()==False):
					objSupervisor.setFlagCheckState(True)
					Timer(1.0, objSupervisor.checkStateMultirotor).start()

				#if(flagCheckState==False):
				#    Timer(1.0, checkStateMultirotor).start()
				#    flagCheckState=True
			
			#Se vuelve a llamar a DoMission en caso que todavia no este activado el modo automatico
			Timer(2.0, self.doMission).start()


	#La siguiente funcion permite al dron a regresar a casa
	def backHome(self):

		global objSupervisor
		#A continuacion seteamos las variables de de FlagControl, InactivityCounter, ChannelModified
		objSupervisor.setFlagControl(False)
		objSupervisor.setInactivityCounter()
		objSupervisor.setChannelModified('',0)
		#Se coloca los canales del 1 al 4 en 1500
		line='rc 1 1500'
		mpstate.input_queue.put(line)
		line='rc 2 1500'
		mpstate.input_queue.put(line)
		line='rc 3 1500'
		mpstate.input_queue.put(line)
		line='rc 4 1500'
		mpstate.input_queue.put(line)
		#El siguiente comando permite activar el modo RTL
		line='RTL'
		mpstate.input_queue.put(line)

	#La siguiente funcion permite activar el dron en Poshold, en este modo de vuelo ya se puede controlar el dron manualmente
	def changePoshold(self):
		print "POSHOLD"
		global objSupervisor
		
		flightMode=str(mpstate.status.flightmode)
		if(flightMode!='POSHOLD'):
			objSupervisor.setFlagControl(False)
			objSupervisor.setChannelModified('',0)
			line='rc 1 0'
			mpstate.input_queue.put(line)
			line='rc 2 0'
			mpstate.input_queue.put(line)
			line='rc 4 0'
			mpstate.input_queue.put(line)		
			line='rc 3 1500'
			mpstate.input_queue.put(line)
			line='POSHOLD'
			mpstate.input_queue.put(line)
			save_log("MESSAGE: Mode poshold activated")


#La siguiente clase permite modificar los parametros de conexion, ip del servidor, puerto del servidor, puerto de la raspberry para la telemeria
#El archivo de configuracion se llama config.json(Este archivo esta dentro de la carpeta ManagementVxRasp)
class Connection(object):

	def __init__(self):

		self.currentVersionMavproxy='vx.v.4'
		self.__ipServer=''
		self.__portServer=''
		self.__ipRaspberry=''
		self.__portRaspTelemetry=''
		self.__stateDebug=''
		self.__outputAdd=''
		self.__setConfiguration()
		self.__serialMultirotor=self.setSerialMultirotor()
		
		print "imprimiendo constructor"

	#Con la siguiente funcion establecemos la configuracion de la ip del servidor, puerto del servidor, puerto de la raspberry para la telemetria
	#ip de la raspberry

	def __setConfiguration(self):
		try:
			data=json.load(open('config.json'))
			self.__ipServer=data["configuration"][0]["ipServer"]
			self.__portServer=data["configuration"][0]["portServer"]
			self.__portRaspTelemetry=data["configuration"][0]["portRaspberryTelemetry"]
			interface=data["configuration"][0]["NetworkInterfaceRaspberry"]
			self.__ipRaspberry=ni.ifaddresses(interface)[2][0]['addr']
			self.__stateDebug=data["configuration"][0]["stateDebug"]
			self.__outputAdd=data["configuration"][0]["outputAdd"]
                        
		except Exception as e:
			print "Problemas al leer el archivo de configuracion"
			print e



	def getIpServer(self):

		return self.__ipServer

	def getPortServer(self):
		return self.__portServer

	def getPortRaspTelemetry(self):
		return self.__portRaspTelemetry

	def getIpRaspberry(self):
		return self.__ipRaspberry

	def getStateDebug(self):
		
		return self.__stateDebug

	def getOutputAdd(self):
		return self.__outputAdd

	def getSerialMultirotor(self):
		print self.__serialMultirotor
		return self.__serialMultirotor

	#La siguiente funcion obtiene el serial de la raspberry
	def setSerialMultirotor(self):
		serialMultirotor = "0000000000000000"
		try:
			f = open('/proc/cpuinfo','r')
			for line in f:
				if line[0:6]=='Serial':
					serialMultirotor = line[10:26]
			f.close()

		except:
			serialMultirotor = "ERROR000000000"

		return serialMultirotor

	#La siguiente funcion permite finalizar el vuelo actual
	def finishCurrentFlight(self):


		ipServer=str(self.getIpServer())
		portServer=str(self.getPortServer())
		global objToken
		
		
		if(self.getSerialMultirotor()!="ERROR000000000"):
			#Se llama al webservice llamado finishOldFlights de Virtuxplorer, el cual recibe como parametros el token del vuelo  y el serial del dron
			url='http://'+ ipServer + ':' + portServer+'/Virtuxplorer/Service/finishOldFlights'
			headers = {'content-type': 'application/json'}
			#En el siguiente payload se adjunta el contenido del json que se envia al webservice de Virtuxplorer
			payload = {'idSerial': self.getSerialMultirotor(),'token':objToken.getLocalToken()}
			r = requests.post(url, data = json.dumps(payload), headers = headers)
			respuesta = str(r.json()['exito'])

			if(respuesta=="1"):
				save_log("MESSAGE: The previous flights have been ssuccessfully completed")
			else:
				save_log("MESSAGE: The previous flights could not finish correctly")
				

	#La siguiente funcion permite finalizar vuelos anteriores diferentes al vuelo actual, por ende, se envia el token actual para cerrar todos los vuelos excepto el actual.
	def finishOldFlights(self):

		global objToken
		ipServer=str(self.getIpServer())
		portServer=str(self.getPortServer())
		if(self.getSerialMultirotor()!="ERROR000000000"):
			#El Webservice para finalizar vuelos pendientes que no se finalizaron correctamente se llama finishOldFlights(Virtuxplorer)

			url='http://'+ ipServer + ':' + portServer+'/Virtuxplorer/Service/finishOldFlights'
			headers = {'content-type': 'application/json'}
			#Al webservice se le envia el idSerial del dron y el token
			payload = {'idSerial': self.getSerialMultirotor(),'token':objToken.getLocalToken()}
			r = requests.post(url, data = json.dumps(payload), headers = headers)
			respuesta = str(r.json()['exito'])
			if(respuesta=="1"):
				save_log("MESSAGE: The previous flights have been ssuccessfully completed")
			else:
				save_log("MESSAGE: The previous flights could not finish correctly")    
   
	#El siguiente metodo se invoca cuando se crea una conexion entre el dron y el cliente, se 
	def initializeConnection(self):

		master=mpstate.master()
		flightMode=str(mpstate.status.flightmode)
		stateArmed=mpstate.master().motors_armed()

		if(flightMode!='GUIDED' and stateArmed==False):
			command='GUIDED'
			mpstate.input_queue.put(command)
		#La siguiente linea configura el FS_BATT_ENABLE en 2, por ende, cuando se esta agotando la bateria el dron regresa a casa
		command='param set FS_BATT_ENABLE 2'
		mpstate.input_queue.put(command)
		#Se configura en cero el siguiente parametro para que la controladora no realice ningun check para poder armar los motores
		command='param set ARMING_CHECK 0'
		mpstate.input_queue.put(command)
		#Se configurar el groundspeed  en 5 m/s
		command='setspeed 5'
		mpstate.input_queue.put(command)
		#Cuando el dron ya esta cerca del home cuando esta en modo RTL, retorna con la misma altitud a la que se encuentra volando el dron
		command='param set RTL_ALT_FINAL 0'
		mpstate.input_queue.put(command)
		#El siguiente parametro configura la velocidad respecto al suelo a 500 cm/seg
		command='param set RTL_SPEED 500'
		mpstate.input_queue.put(command)
		#Cuando el dron esta retornando a casa, retorna con la mimsa altitud a la que se encuentra
		command='param set RTL_ALT 0'
		mpstate.input_queue.put(command)
		save_log("MESSAGE: Initialize connection")
	
	#El siguiente metodo activa el dron en Virtuxplorer(es decir, lo coloca como disponible), este metodo se llama cuando se inicia el dron por primera vez o cuando se finaliza el vuelo
	def activateMultirotor(self):
		print("activate multirotor")
		ipServer=str(self.getIpServer())
		portServer=str(self.getPortServer())
		serialMultirotor=(self.getSerialMultirotor())
		print serialMultirotor
		if(serialMultirotor!="ERROR000000000"):
	
			url='http://'+ ipServer + ':' + portServer+'/Virtuxplorer/Service/activateMultirotor'
			headers = {'content-type': 'application/json'}
			payload = {'idSerial': serialMultirotor}
			r = requests.post(url, data = json.dumps(payload), headers = headers)
			respuesta = str(r.json()['exito'])
	
			if(respuesta=="1"):
				save_log("MESSAGE: The multirotor have been succesfully enabled")
			else:
				save_log("MESSAGE: The multirotor could not be disabled")
				
	#Esta funcion elimina los procesos que esten ocupando el puerto para la telemetria
	def killProcess(self):
	
		netstat = subprocess.Popen('netstat -lpn'.split() , stdout=subprocess.PIPE)
		portNumber=str(self.getPortRaspTelemetry())
		print "kill"
		print portNumber
		grep = subprocess.Popen(('grep :'+portNumber).split(), stdin=netstat.stdout, stdout=subprocess.PIPE)
		output = grep.communicate()[0]
		patron=re.compile('[1-9][0-9]*/python')
		m = patron.findall(output)
		for i in range(0,len(m)):
			port = m[i][0:m[i].find('/')]
			print port
			subprocess.call(["kill","-9",port])
	
	#Esta funcion desactiva el video , finaliza el vuelo actual y activa el multirotor para poder ejecutar vuelos.
	def restablecerDron(self):

		global objToken
		print "Restableciendo dron"
		objCamera=Camera()
		objCamera.disableVideo()
		self.finishCurrentFlight()
		self.activateMultirotor()
		objToken=Token('',False)

#La siguiente clase posee funciones para mover la camara hacia arriba,abajo, centrar la camara, activar el video y desactivar el video
class Camera(object):

	def __init__(self):
		self.enableVideo()

	#La siguiente funcion permite mover la camara hacia arriba
	def moveCameraUp(self):
		#channel6=int(master.field('RC_CHANNELS_RAW', 'chan6_raw', 0))
		line='rc 6 1400'
		mpstate.input_queue.put(line)   

	#La siguiente funcion permite mover la camara hacia abajo
	def moveCameraDown(self):

		#channel6=int(master.field('RC_CHANNELS_RAW', 'chan6_raw', 0))
		line='rc 6 2000'
		mpstate.input_queue.put(line)   
	#La siguiente funcion permite centrar la camara del dron
	def centerCamera(self):

		#channel6=int(master.field('RC_CHANNELS_RAW', 'chan6_raw', 0))
		if(channel6!=1500):
			line='rc 6 1500 '
			mpstate.input_queue.put(line)
	#La siguiente funcion permite activar el video con el servicio uv4l
	def enableVideo(self):
		p = os.popen('sudo service uv4l_raspicam start')
	#La siguiente funcion desactiva el servicio de video de transmision de video de uv4l
	def disableVideo(self):
		p = os.popen('sudo service uv4l_raspicam stop')   


#La siguiente clase contiene los metodos para calcular la distancia al fence, saber la distancia a las zonas de conflicto, inicializar parametros del fence
class Fence(object):

	#Constructor de la clase Fence
	def __init__(self, heightMin,heightMax,listCoordinates,listRectangles, listCommands, listRally):
		self.master=mpstate.master()
		self.heightMin=heightMin
		self.heightMax=heightMax
		self.positionHome=str(self.master.messages['HOME'])
		self.listPointFence=[]
		self.listRectangles=[]
		self.heightRectangleNear=0
		self.createPointPolygon(listCoordinates)
		self.createFileFence(listCoordinates)
		self.initializeGeofence()
		self.createRectangles(listRectangles)
		self.objMission=Mission(listCommands, self.positionHome)
		self.objRally=Rally(listRally)
		
		save_log("MESSAGE: setListRectangles")
		save_log("MESSAGE: setMission")
		save_log("MESSAGE: setRally")


	#La siguiente funcion retorna la altura minima de la zona
	def getHeightMin(self):

		return self.heightMin

	#La siguiente funcion configura los parametros basicos del fence
	def initializeGeofence(self):

		#El siguiente parametro carga el modulo del fence
		command='module load fence'
		mpstate.input_queue.put(command)
		#El siguiente parametro activa el fence
		command='param set FENCE_ENABLE 1'
		mpstate.input_queue.put(command)
		#El siguiente parametro carga el archivo del fence
		command='fence load fence.txt'
		mpstate.input_queue.put(command)
		command='fence enable'      
		mpstate.input_queue.put(command)
		#El  siguiente parametro configura el 
		command='param set FENCE_ACTION 1'
		mpstate.input_queue.put(command)
		command='param set FENCE_TYPE 5'
		mpstate.input_queue.put(command)
		command='param set FENCE_ALT_MAX '+str(self.heightMax)
		mpstate.input_queue.put(command)
		command='param set FENCE_MARGIN 2'
		mpstate.input_queue.put(command)
		save_log("MESSAGE: Params fence established")


	def checkInsideRectangle(self):

		flagInside=False
		master=mpstate.master()
		latitud=float(master.field('GLOBAL_POSITION_INT', 'lat', 0) * 1.0e-7)
		longitud=float(master.field('GLOBAL_POSITION_INT', 'lon', 0) * 1.0e-7)
		altitud=float(master.field('GLOBAL_POSITION_INT', 'relative_alt',0)* 1.0e-3)

		for objRectangle in self.listRectangles:

			if((latitud>=objRectangle.listVertex[1].latitude) and (latitud<=objRectangle.listVertex[0].latitude) and (longitud>=objRectangle.listVertex[1].longitude) and (longitud<=objRectangle.listVertex[2].longitude) and (altitud<objRectangle.heightRectangle)):
				print "INSIDE RECTANGLE AND ALTITUDE IS LOW"
				save_log("WARNING: The drone is inside the rectangle and the altitude is below that of the rectangle")
				flagInside=True
		
			else:
				flagInside=False            
		
		return flagInside


	def getDistanceFence(self):

		distTotal=99999
		lineStartLatLng=None
		radioTierra=6371e3
		master=mpstate.master()
		latitud=float(master.field('GLOBAL_POSITION_INT', 'lat', 0) * 1.0e-7)
		longitud=float(master.field('GLOBAL_POSITION_INT', 'lon', 0) * 1.0e-7)
		objCurrentLocation=PointFence(latitud,longitud)

		for x in self.listPointFence:

			if(lineStartLatLng==None):
				lineStartLatLng=PointFence(x.latitude,x.longitude)
				continue

			lineEndLatLng=PointFence(x.latitude,x.longitude)
			lineDist=lineStartLatLng.getDistance2(lineEndLatLng)
			distToLocation=lineStartLatLng.getDistance2(objCurrentLocation)
			bearToLocation = lineStartLatLng.getBearing(objCurrentLocation)	    
			lineBear = lineStartLatLng.getBearing(lineEndLatLng)
			angle = bearToLocation - lineBear

			if (angle < 0):
				angle += 360
			alongline = math.cos(math.radians(angle))*distToLocation
			
			if (alongline > lineDist):
				lineStartLatLng = lineEndLatLng
				continue

			dXt2 = math.sin(math.radians(angle))*distToLocation
			dXt = math.asin(math.sin(distToLocation/radioTierra)*math.sin(math.radians(angle)))*radioTierra
			distTotal = float(min(distTotal, abs(dXt2)))     
			lineStartLatLng = lineEndLatLng

		for x in self.listPointFence:
			dXt2 = x.getDistance(objCurrentLocation)
		distTotal = float(min(distTotal,abs(dXt2)))

		return distTotal

	#Este metodo se utiliza para calcular la distancia al rectangulo mas cercano
	def checkDistanceRectangles(self):

		distTotal=999999
		distFinal=999999
		lineStartLatLng=None
		radioTierra=6371e3
		master=mpstate.master()
		latitud=float(master.field('GLOBAL_POSITION_INT', 'lat', 0) * 1.0e-7)
		longitud=float(master.field('GLOBAL_POSITION_INT', 'lon', 0) * 1.0e-7)
		objCurrentLocation=PointFence(latitud,longitud)

		for objRectangle in self.listRectangles:
			for x in objRectangle.listPointRectangle:
				multiplierdist=1
				rads=abs(x.latitude)*0.0174532925
				scaleLongDown=float(math.cos(rads))
				scaleLongUp=1/(math.cos(rads))
				dstlat=abs(x.latitude-latitud)*111319.5
				dstlon=abs(x.longitude-longitud)*111319.5*scaleLongDown
				distToPoint=float(math.sqrt((dstlat*dstlat)+(dstlon*dstlon)*multiplierdist))
		
				if(distToPoint<distFinal):
					distFinal=distToPoint    
					self.heightRectangleNear=objRectangle.heightRectangle+1
				
			   
		return distFinal        

	def getHeightRectangleNear(self):
		return self.heightRectangleNear


	def createPointPolygon(self,listCoordinates):
		latitude1=0
		longitude1=0
		aux=0

		for x in listCoordinates:
			objPointFence=PointFence(float(x['latitud']),float(x['longitud']))
			self.listPointFence.append(objPointFence)

		save_log("MESSAGE: Geofence created")


	def createRectangles(self,listRectanglesReceived):

		for x in listRectanglesReceived:

			listPointsCurrentRectangle=[]
			listVertex=[]
			diffEastWest=(float(x['East'])-float(x['West']))/8

			for i in range (0,9):
				acum=float(x['West'])+(i*diffEastWest)
				objPointRectangle=PointFence(float(x['North']),acum)   
				listPointsCurrentRectangle.append(objPointRectangle)

			diffNorthSouth=(float(x['North'])-float(x['South']))/8        
			
			for i in range (0,9):
				acum=float(x['South'])+(i*diffNorthSouth)
				objPointRectangle=PointFence(acum,float(x['West']))   
				listPointsCurrentRectangle.append(objPointRectangle)
			
			for i in range (0,9):
				acum=float(x['West'])+(i*diffEastWest)
				objPointRectangle=PointFence(float(x['South']),acum)   
				listPointsCurrentRectangle.append(objPointRectangle)

			for i in range (0,9):
				acum=float(x['South'])+(i*diffNorthSouth)
				objPointRectangle=PointFence(acum,float(x['East']))   
				listPointsCurrentRectangle.append(objPointRectangle)

			objVertex=PointFence(float(x['North']),float(x['West']))
			listVertex.append(objVertex)

			objVertex=PointFence(float(x['South']),float(x['West']))
			listVertex.append(objVertex)
 
			objVertex=PointFence(float(x['South']),float(x['East']))
			listVertex.append(objVertex)

			objVertex=PointFence(float(x['North']),float(x['East']))
			listVertex.append(objVertex)

			heightRectangle=float(x['Height'])
			objRectangle=Rectangle(listPointsCurrentRectangle,listVertex,heightRectangle)
			self.listRectangles.append(objRectangle)

		save_log("MESSAGE: Conflict areas created")

	def createFileFence(self,listCoordinates):

		fo = open("fence.txt", "wb")
		fo.write("#Saved by Virtuxplorer\n")
		posLat=self.positionHome.index("lat")
		posLon=self.positionHome.index("lon")
		posAlt=self.positionHome.index("alt")
		latHome=float(self.positionHome[posLat+6:posLon-2])* 1.0e-7
		lonHome=float(self.positionHome[posLon+6:posAlt-2])* 1.0e-7
		fo.write( str(latHome)+'\t'+'\t'+str(lonHome)+'\n')
		
		for x in listCoordinates:
			fo.write( x['latitud']+'\t'+x['longitud']+'\n')
		fo.write(str(listCoordinates[0]['latitud'])+'\t'+str(listCoordinates[0]['longitud']))
		fo.close()
		save_log("MESSAGE: File fence created")
            


class Mission(object):



	def __init__(self, listCommands, positionHome):
		self.createFileMission(listCommands,positionHome)

	def setMission(self):

		command='wp load missionVirtuxplorer.txt'
		mpstate.input_queue.put(command)
		save_log("MESSAGE: Mission loaded")

	def createFileMission(self,listCommands, positionHome):

		indexWaypoint=1
		posLat=positionHome.index("lat")
		posLon=positionHome.index("lon")
		posAlt=positionHome.index("alt")
		latHome=float(positionHome[posLat+6:posLon-2])* 1.0e-7
		lonHome=float(positionHome[posLon+6:posAlt-2])* 1.0e-7
		altHome=50
		fo=open("missionVirtuxplorer.txt","wb")
		fo.write('QGC'+' '+'WPL'+' '+'110'+'\n')
		fo.write('0'+'\t'+'1'+'\t'+'0'+'\t'+'16'+'\t'+'0'+'\t'+'0'+'\t'+'0'+'\t'+'0'+'\t'+str(latHome)+'\t'+str(lonHome)+'\t'+str(altHome)+'\t'+'1'+'\n')
		for x in listCommands:
			fo.write(str(indexWaypoint)+'\t'+'0'+'\t'+'3'+'\t'+x['commandId']+'\t'+x['param1']+'\t'+x['param2']+'\t'+x['param3']+'\t'+x['param4']+'\t'+x['latitude']+'\t'+x['longitude']+'\t'+x['altitude']+'\t'+'1'+'\n')
			indexWaypoint+=1
		fo.close()
		save_log("MESSAGE: File mission created")
		self.setMission()

class PointFence:


    def __init__(self, latitude1,longitude1):
        self.latitude=latitude1
        self.longitude=longitude1
    def getDistance2(self,p2):
        radioTierra=6371
        dLat=math.radians(p2.latitude-self.latitude)
       
        dLon=math.radians(p2.longitude-self.longitude)
        lat1=math.radians(self.latitude)
        lat2=math.radians(p2.latitude)
        a=math.sin(dLat/2)*math.sin(dLat/2)+math.sin(dLon/2)*math.sin(dLon/2)*math.cos(lat1)*math.cos(lat2)
        c=2*math.atan2(math.sqrt(a), math.sqrt(1-a))
        d=radioTierra*c*1000
       
        return d

    def getBearing(self,p2):
	latitude1 = math.radians(self.latitude)
        latitude2 = math.radians(p2.latitude)
        longitudeDifference = math.radians(p2.longitude - self.longitude)
        y = math.sin(longitudeDifference) * math.cos(latitude2)
        x = math.cos(latitude1) * math.sin(latitude2) - math.sin(latitude1) * math.cos(latitude2) * math.cos(longitudeDifference)
        return (math.degrees(math.atan2(y, x)) + 360) % 360
    
    def getDistance(self, p2):
    	d = self.latitude * 0.017453292519943295
        num2 = self.longitude * 0.017453292519943295
        num3 = p2.latitude * 0.017453292519943295
        num4 = p2.longitude * 0.017453292519943295
        num5 = num4 - num2
        num6 = num3 - d
        num7 = math.pow(math.sin(num6 / 2.0), 2.0) + ((math.cos(d) * math.cos(num3)) * math.pow(math.sin(num5 / 2.0), 2.0))
        num8 = 2.0 * math.atan2(math.sqrt(num7), math.sqrt(1.0 - num7))
        return (6371 * num8) * 1000.0


class Rally(object):

	def __init__(self,listRally):
		self.createFileRally(listRally)

	def setRally(self):

		command='rally load listRally.txt'
		mpstate.input_queue.put(command)
		save_log("MESSAGE: Rally loaded")

	def createFileRally(self,listRally):

		fo=open("listRally.txt","wb")
		fo.write('#Saved by Virtuxplorer'+'\n')
		for x in listRally:
			fo.write('RALLY'+'\t'+x['latitude']+'\t'+x['longitude']+'\t'+x['altitude']+'\t'+'0'+'\t'+'0'+'\t'+'0'+'\n')

		fo.close()
		self.setRally()
		save_log("MESSAGE: File rally created")
      


class Rectangle:
    listPointRectangle=[]
    listVertex=[]
    heightRectangle=0

    def __init__(self, listPoints,listVertex,height):
        self.listPointRectangle=listPoints
        self.listVertex=listVertex
        self.heightRectangle=height
    


class Supervisor(object):

	def __init__(self):

		self.flagControl=False
		self.flagCheckDisarmed=False
		self.flagCheckState=False
		self.timeInactivityVirtuxplorer=0
		self.flagCheckConnectionVirtuxplorer=False
		self.flagFence=0
		self.lastLatitude=0
		self.lastLongitude=0
		self.lastAltitude=0
		self.flagObstacleNear=False
		self.flagObstacleInside=False
		self.banderaFence=False
		self.fenceMinAltitude=0
		self.nameChannelModified=''
		self.valueChannelModified=0

	def setFlagCheckState(self, aux):
		self.flagCheckState=aux

	def getFlagCheckState(self):
		return self.flagCheckState

	def getFlagCheckDisarmed(self):
		return self.flagCheckDisarmed

	def setFlagCheckDisarmed(self,aux):
		self.flagCheckDisarmed=aux

	def getFlagControl(self):
		return self.flagControl

	def setFlagControl(self,aux):
		self.flagControl=aux

	def getFlagCheckInactivity(self):
		return self.flagCheckInactivity

	def setFlagCheckInactivity(self,aux):
		self.flagCheckInactivity=aux

	def getInactivityCounter(self):
		return self.inactivityCounter

	def setInactivityCounter(self):
		self.inactivityCounter=0

	def getTimeInactivityVirtuxplorer(self):
		return self.timeInactivityVirtuxplorer

	def setTimeInactivityVirtuxplorer(self):
		self.timeInactivityVirtuxplorer=0

	def getFlagCheckConnectionVirtuxplorer(self):
		return self.flagCheckConnectionVirtuxplorer

	def setFlagCheckConnectionVirtuxplorer(self,aux):
		self.flagCheckConnectionVirtuxplorer=aux


	def getFlagFence(self):
		return self.flagFence

	def setFlagFence(self,aux):
		self.flagFence=aux

	def getLastLatitude(self):
		return self.lastLatitude

	def setLastLatitude(self,aux):
		self.lastLatitude

	def getLastLongitude(self):
		return self.lastLongitude

	def setLastLongitude(self,aux):
		self.lastLongitude=aux

	def getLastAltitude(self):
		return self.lastAltitude

	def setLasAltitude(self,aux):
		self.lastAltitude=aux

	def getFlagObstacleNear(self):
		return self.flagObstacleNear

	def setFlagObstacleNear(self,aux):
		self.flagObstacleNear=aux

	def getFlagObstacleInside(self):
		return self.flagObstacleInside

	def setFlagObstacleInside(self,aux):
		self.flagObstacleInside=aux

	def getBanderaFence(self):
		return self.banderaFence

	def setBanderaFence(self,aux):
		self.banderaFence=aux

	def getFenceMinAltitude(self):
		return self.fenceMinAltitude

	def setFenceMinAltitude(self,aux):
		self.fenceMinAltitude=aux

	def getNameChannelModified(self):
		return self.nameChannelModified

	def setChannelModified(self,nameChannel,valueChannel):
		self.nameChannelModified=nameChannel
		self.valueChannelModified=valueChannel

	def getValueChannelModified(self):
		return self.valueChannelModified




#La siguiente funcion se utiliza para detectar perdida de conexion de parte de Virtuxplorer,
#Si se detecta inactividad por mas de 20 segundos, se regresa a casa en el modo de vuelo manual
#Regresa a casa cuando la inactividad es mayor a 60 segundos en el modo de vuelo automatico
            
	def checkConnectivityVirtuxplorer(self):

		self.timeInactivityVirtuxplorer +=1
		print "checkConnectivity"

		master=mpstate.master()
		flightMode=str(mpstate.status.flightmode)
		stateArmed=mpstate.master().motors_armed()

		if(self.timeInactivityVirtuxplorer >= 20 and self.flagCheckConnectionVirtuxplorer==True and (flightMode=="POSHOLD" or flightMode=="GUIDED" or flightMode=="AUTO")):

			print("##########Se enviar a modo RTL poque no hay conexion con Virtuxplorer")
			save_log("WARNING: Will be sent to RTL mode due to  connection loss with Virtuxplorer") 
			master=mpstate.master()
			mpstate.input_queue.put("RTL")
			self.timeInactivityVirtuxplorer=0
			self.flagCheckConnectionVirtuxplorer=False


	#El siguiente metodo cambia el dron de modo GUIDED a modo POSHOLD cuando el dron se encuentra despegando y sobrepasa cierta altura
	def checkAltitudeTakeoff(self):
		print "CHECK ALTITUDE"
		global objFence
		objFlight=FlightAction()
		heightMin=objFence.getHeightMin()
		master=mpstate.master()
		flightMode=str(mpstate.status.flightmode)
		altitud=float(master.field('GLOBAL_POSITION_INT', 'relative_alt',0)* 1.0e-3)
		
		if(altitud> heightMin and flightMode=="GUIDED"):
			objFlight.changePoshold()
			save_log("MESSAGE: The minimum altitude has been reached")



	# El siguiente metodo verifica el estado del dron cuando intenta sobrepasar el fence, si esta por debajo de la altura minima, si esta adentro o cerca de un obstaculo

	def checkStateMultirotor(self):

     
		global objFence
		global objConnection
		heightMin=objFence.getHeightMin()
		print "CHECK STATE MULTIROTOR"
		master=mpstate.master()
		flightMode=str(mpstate.status.flightmode)
		flagInside=False
		channel1=int(master.field('RC_CHANNELS_RAW', 'chan1_raw', 0))
		channel2=int(master.field('RC_CHANNELS_RAW', 'chan2_raw', 0))
		channel3=int(master.field('RC_CHANNELS_RAW', 'chan3_raw', 0))
		channel4=int(master.field('RC_CHANNELS_RAW', 'chan4_raw', 0))
		stateArmed=mpstate.master().motors_armed()
		altitud=float(master.field('GLOBAL_POSITION_INT', 'relative_alt',0)* 1.0e-3)


		if(self.flagCheckConnectionVirtuxplorer==True and (flightMode=="GUIDED" or flightMode=="POSHOLD" or flightMode=="AUTO")):
			self.checkConnectivityVirtuxplorer()


		if(flightMode=="GUIDED" and stateArmed==True and altitud>heightMin and self.flagFence==0 and self.banderaFence==False):
			self.checkAltitudeTakeoff()


		elif((flightMode=="POSHOLD" or (flightMode=="GUIDED" and self.flagFence==1) or (flightMode=="RTL" and self.flagFence==1)) and stateArmed==True):
			altitud=float(master.field('GLOBAL_POSITION_INT', 'relative_alt',0)* 1.0e-3)
			print altitud
			print heightMin
				
			
			failBattery = str(master.field('STATUSTEXT','text'))
			latitud=float(master.field('GLOBAL_POSITION_INT', 'lat', 0) * 1.0e-7)
			longitud=float(master.field('GLOBAL_POSITION_INT', 'lon', 0) * 1.0e-7)
			distanceFence=objFence.getDistanceFence()

			if(distanceFence>=16):

				self.lastLatitude=latitud
				self.lastLongitude=longitud
				self.lastAltitude=int(math.floor(altitud))

			distanceRectangleNear=objFence.checkDistanceRectangles()

			if(distanceRectangleNear<=5 and flightMode=="POSHOLD" and altitud<objFence.getHeightRectangleNear() and channel3!=1800):

				line='rc 1 1500'
				mpstate.input_queue.put(line)
				line='rc 2 1500'
				mpstate.input_queue.put(line)
				line='rc 3 1800'
				mpstate.input_queue.put(line)
				self.flagControl=False
				self.flagObstacleNear=True
				save_log("WARNING: The drone is near an area of conflict")


			elif(distanceRectangleNear<=5 and altitud>=objFence.getHeightRectangleNear() and self.flagObstacleNear==True and flightMode=="POSHOLD"):
				line='rc 3 1500'      
				mpstate.input_queue.put(line)
				self.flagObstacleNear=False
				save_log("MESSAGE: The altitude of the drone is already greather than the area of conflict")

			elif(distanceRectangleNear>5 and self.flagObstacleNear==True):
				self.flagObstacleNear=False
				save_log("MESSAGE: The drone has moved away from the conflict area")

			flagInside=objFence.checkInsideRectangle() 

			if(flagInside==True and channel3!=1800):
				line='rc 1 1500'
				mpstate.input_queue.put(line)
				line='rc 2 1500'
				mpstate.input_queue.put(line)
				line='rc 3 1800'
				mpstate.input_queue.put(line)
				self.flagControl=False
				self.flagObstacleInside=True
				save_log("WARNING: The accelerator value will be increased because it is within a conflict area")

			elif(flagInside==False and self.flagObstacleInside==True):
				self.flagObstacleInside=False    
				line='rc 3 1500'
				mpstate.input_queue.put(line)
				save_log("MESSAGE:The altitude of the drone is greather than that of the conflict area")

			if(distanceFence<15 and self.banderaFence==False):

				line='GUIDED' 
				mpstate.input_queue.put(line)
				line='rc 1 1500'
				mpstate.input_queue.put(line)
				line='rc 2 1500'
				mpstate.input_queue.put(line)
				line='rc 3 1500'
				mpstate.input_queue.put(line)
				line='rc 4 1500'
				mpstate.input_queue.put(line)
				print "El dron se ha acercado al limite de la zona"
				line='guided ' + str(self.lastLatitude) +" " + str(self.lastLongitude) + " " + str(self.lastAltitude)
				mpstate.input_queue.put(line)
				self.flagFence=1
				self.banderaFence=True
				self.flagControl=False
				save_log("WARNING: The drone is close to the limit of the zone, it will move away from the limit of the zone")

			elif(distanceFence>=15 and self.banderaFence==True):

				line='rc 1 0'
				mpstate.input_queue.put(line)
				line='rc 2 0'
				mpstate.input_queue.put(line)
				line='rc 3 1500'
				mpstate.input_queue.put(line)
				line='rc 4 0'
				mpstate.input_queue.put(line)
				line='POSHOLD'
				mpstate.input_queue.put(line)
				self.banderaFence=False	
				self.flagFence=0
				save_log("MESSAGE: The drone has left the boundary of the zone")


			if(altitud<heightMin and (flightMode=="POSHOLD") and channel3!=1800):

				line='rc 3 1800'
				mpstate.input_queue.put(line)
				line='rc 1 1500'
				mpstate.input_queue.put(line)
				line='rc 2 1500'
				mpstate.input_queue.put(line)
				line='rc 4 1500'
				mpstate.input_queue.put(line) 								
				print "el dron esta por debajo de la altura minima, se elevara su altura"
				self.fenceMinAltitude=1
				flagControl=False    
				save_log("WARNING: The drone is below the minimum height of the zone, the throttle value will be increased")
					  
			elif(altitud>=heightMin and self.fenceMinAltitude==1 and flightMode=="POSHOLD"):

				print "altura del dron restablecida"
				line='rc 3 1500'
				mpstate.input_queue.put(line)
				line='rc 1 0'
				mpstate.input_queue.put(line)
				line='rc 2 0'
				mpstate.input_queue.put(line)
				line='rc 4 0'
				mpstate.input_queue.put(line)
				self.fenceMinAltitude=0
				save_log("MESSAGE: The altitude of the drone is already greather than the minimum altitude of the zone")


			if(failBattery=='Low battery'):
				line='RTL'
				self.flagControl=False
				mpstate.input_queue.put(line)
				save_log("The level battery is low, the dron will return to home")

		elif((flightMode=="LAND" or flightMode=="RTL" or flightMode=="AUTO") and (channel1!=1500 or channel2!=1500 or channel3!=1500 or channel4!=1500) and stateArmed==True):

			self.nameChannelModified=''
			line='rc 1 1500'
			mpstate.input_queue.put(line)
			line='rc 2 1500'
			mpstate.input_queue.put(line)
			line='rc 3 1500'
			mpstate.input_queue.put(line)      
			line='rc 4 1500'
			mpstate.input_queue.put(line)
			save_log("MESSAGE: Equilibrando canales en modo "+flightMode)
			self.flagControl=False

		stateArmed=mpstate.master().motors_armed()    

		if((flightMode=="RTL"  or flightMode=="AUTO") and stateArmed==False and self.flagCheckDisarmed==False):


			self.flagCheckDisarmed=True
			self.flagCheckConnectionVirtuxplorer=False
			self.setFlagCheckState(False)
			self.timeInactivityVirtuxplorer=0
			objConnection.restablecerDron()
			
		#elif((stateArmed==True and (flightMode=="POSHOLD" or flightMode=="RTL" or flightMode=="AUTO" or flightMode=="GUIDED")) or (stateArmed==False and flightMode=="GUIDED")): 
		#	Timer(1.0, self.checkStateMultirotor).start()
		if(self.flagCheckState==True):
			Timer(1.0, self.checkStateMultirotor).start()

	def checkGroundSpeed(self):


		master=mpstate.master()
		groundSpeed=float(master.field('VFR_HUD','groundspeed'))
		flightMode=str(mpstate.status.flightmode)
		channel1=int(master.field('RC_CHANNELS_RAW', 'chan1_raw', 0))	
		channel2=int(master.field('RC_CHANNELS_RAW', 'chan2_raw', 0))
		
		
		if(self.nameChannelModified!='' and flightMode=="POSHOLD" and self.flagControl==True):


			if(groundSpeed>3 and flightMode=="POSHOLD"):

				save_log("WARNING: The groundspeed is "+ str(groundSpeed))

				if(self.nameChannelModified=='rollLeft' and channel1==self.valueChannelModified):

					self.valueChannelModified=self.valueChannelModified+25
					line='rc 1 '+str(self.valueChannelModified)
					mpstate.input_queue.put(line)
					print "modificando roll Left"
					save_log("WARNING: Modifying roll left to value: "+str(self.valueChannelModified))

				elif(self.nameChannelModified=='rollRight' and channel1==self.valueChannelModified):

					 self.valueChannelModified=self.valueChannelModified-25
					 line='rc 1 '+str(self.valueChannelModified)
					 mpstate.input_queue.put(line)
					 print "modificnado roll Right"
					 save_log("WARNING: Modifying roll right to value: "+str(self.valueChannelModified))
		
				elif(self.nameChannelModified=='pitchFront' and channel2==self.valueChannelModified):
		
					 self.valueChannelModified=self.valueChannelModified+25
					 line='rc 2 '+str(self.valueChannelModified)
					 mpstate.input_queue.put(line)
					 print "modificando pitch Front"
					 save_log("WARNING: Modifying pitch front to value: "+str(self.valueChannelModified))
		
				elif(self.nameChannelModified=='pitchBack' and channel2==self.valueChannelModified):

					 self.valueChannelModified=self.valueChannelModified-25
					 line='rc 2 '+str(self.valueChannelModified)
					 mpstate.input_queue.put(line)
					 print "modificando pitchBack"
					 save_log("WARNING: Modifying pitch back to value: "+str(self.valueChannelModified))


			elif(groundSpeed<1 and flightMode=="POSHOLD"):
		
				save_log("WARNING: The groundspeed is "+ str(groundSpeed))
				if(self.nameChannelModified=='rollLeft' and channel1==self.valueChannelModified):
		
					self.valueChannelModified=self.valueChannelModified-25
					line='rc 1 '+str(self.valueChannelModified)
					mpstate.input_queue.put(line)
					save_log("WARNING: Modifying roll left to value: "+str(self.valueChannelModified))
		
				elif(self.nameChannelModified=='rollRight'  and channel1==self.valueChannelModified):
		
					self.valueChannelModified=self.valueChannelModified+25
					line='rc 1 '+str(self.valueChannelModified)
					mpstate.input_queue.put(line)
					save_log("WARNING: Modifying roll right to value: "+str(self.valueChannelModified))
					 
		
				elif(self.nameChannelModified=='pitchFront' and channel2==self.valueChannelModified):

					self.valueChannelModified=self.valueChannelModified-25
					line='rc 2 '+str(self.valueChannelModified)
					mpstate.input_queue.put(line)
					save_log("WARNING: Modifying pitch front to value: "+str(self.valueChannelModified))

				elif(self.nameChannelModified=='pitchBack' and channel2==self.valueChannelModified):

					self.valueChannelModified=self.valueChannelModified+25
					line='rc 2 '+str(self.valueChannelModified)
					mpstate.input_queue.put(line)
					save_log("WARNING: Modifying pitch back to value: "+str(self.valueChannelModified))

		elif((flightMode=="POSHOLD" and self.nameChannelModified=='') or flightMode!="POSHOLD" or self.flagControl==False):

			self.nameChannelModified==''
			line='rc 1 1500'
			mpstate.input_queue.put(line)
			line='rc 2 1500'
			mpstate.input_queue.put(line)
			line='rc 1 0'
			mpstate.input_queue.put(line)
			line='rc 2 0'
			mpstate.input_queue.put(line)
			save_log("WARNING: Balancing value channels roll and pitch")
		
		if(flightMode=="POSHOLD" and self.nameChannelModified!='' and self.flagControl==True):
			Timer(3.0,self.checkGroundSpeed).start()


class Token(object):

	def __init__(self,localToken,validToken):
		self.localToken=localToken
		self.isValidToken=validToken

	def getLocalToken(self):
		return self.localToken

	def getIsValidToken(self):
		return self.isValidToken
        

def calculateDistanceHome(latCurrent,lngCurrent):
    master=mpstate.master()
    multiplierdist=1
    positionHome=str(master.messages['HOME'])
    posLat=positionHome.index("lat")
    posLon=positionHome.index("lon")
    posAlt=positionHome.index("alt")
    latHome=float(positionHome[posLat+6:posLon-2])* 1.0e-7
    lonHome=float(positionHome[posLon+6:posAlt-2])* 1.0e-7
    rads=abs(latHome)*0.0174532925
    scaleLongDown=float(math.cos(rads))
    scaleLongUp=1/(math.cos(rads))
    dstlat=abs(latHome-latCurrent)*111319.5
    dstlon=abs(lonHome-lngCurrent)*111319.5*scaleLongDown
    return float(math.sqrt((dstlat*dstlat)+(dstlon*dstlon)*multiplierdist))         
                





#@@@@@@@ FUNCIONES PARA INICIAR Y OBTENER EL TIEMPO DE VUELO @@@@@@@@
         
def startTimeFlight():

    global flagSetChronoTime
    if(flagSetChronoTime==False):
        setChronoTime()    
        flagSetChronoTime=True

def setChronoTime():

    global timeFlight
    timeFlight+=1
    Timer(1.0,setChronoTime).start()


@app.route('/getFlightTime',methods=['POST'])
def getFlightTime():

    global timeFlight    
    payload={'timeCurrentFlight':timeFlight}
    data=json.dumps(payload)
    return data
        
#@@@@@@@ FIN DE FUNCIONES PARA INICIAR Y OBTENER EL TIEMPO DE VUELO @@@@@@@@






#@@@@@@@ FUNCIONES PARA CREAR LOS ARCHIVOS DE LOG @@@@@@@@@@@@@@         

def save_log(mensaje):

    hora_actual=str(datetime.datetime.now())
    flog=open("logTelemetryVirtuxplorer.txt","a")
    flog.write(hora_actual+" "+mensaje+"\n")
    flog.close()
    
    




#@@@@@@@@@@@ FUNCIONES DE TELEMETRIA  @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
         

@app.route('/getPositionHome',methods=['POST'])
def getPositionHome():
    
    master=mpstate.master()
    positionHome=str(master.messages['HOME'])
    posLat=positionHome.index("lat")
    posLon=positionHome.index("lon")
    posAlt=positionHome.index("alt")
    latHome=float(positionHome[posLat+6:posLon-2])* 1.0e-7
    lonHome=float(positionHome[posLon+6:posAlt-2])* 1.0e-7
    payload={'latHome':latHome,'lonHome':lonHome}
    data=json.dumps(payload)
    return data


def writeNetworkLog():
	type_request = master.field('DATA_TRANSMISSION_HANDSHAKE', 'type', 0)
	print type_request
	
	


@app.route('/getTelemetryVirtuxplorer',methods=['POST'])
def getTelemetry():



	global objConnection
	global objSupervisor
	global objToken

	ipServer=str(objConnection.getIpServer())
	portServer=str(objConnection.getPortServer())
    

	flagObstacle=False
	incomingToken = request.json['token']

	if request.json:

		if(objToken.getIsValidToken()==True and (objToken.getLocalToken() == incomingToken)):

			idFlight=request.json['idFlight']
			objSupervisor.setTimeInactivityVirtuxplorer()
			url='http://'+ ipServer + ':' + portServer+'/Virtuxplorer/TelemetryWebservice/receiveTelemetry'
			headers = {'content-type': 'application/json'}
			
			master=mpstate.master()
			flightMode=str(mpstate.status.flightmode)    
			stateArmed=mpstate.master().motors_armed()
			lat = master.field('GLOBAL_POSITION_INT', 'lat', 0) * 1.0e-7
			lng = master.field('GLOBAL_POSITION_INT', 'lon', 0) * 1.0e-7
			altitud=float(master.field('GLOBAL_POSITION_INT', 'relative_alt',0)* 1.0e-3)
			angle = master.field('VFR_HUD','heading')
			distanceHome=calculateDistanceHome(float(lat),float(lng))
			fence=str(objSupervisor.getFlagFence())
			failBattery = master.field('STATUSTEXT','text')
			battery = master.field('SYS_STATUS','current_battery',0)
			distanceTarget = master.field('NAV_CONTROLLER_OUTPUT','wp_dist')
			numberWaypoint = master.field('MISSION_CURRENT','seq',0)
			hdop = master.field('GPS_RAW_INT','eph')*1.0e-2
			fixType = master.field('GPS_RAW_INT','fix_type',0)
			numberSatellites = master.field('GPS_RAW_INT','satellites_visible')
			airSpeed = round(master.field('VFR_HUD', 'airspeed'),2)
			command_ack=master.field('COMMAND_ACK','command',0)
			
			if(objSupervisor.getFlagObstacleNear()==True or objSupervisor.getFlagObstacleInside()==True):
				flagObstacle=True
			else:
				flagObstacle=False

			payload = {'idFlight': idFlight,'latitude': lat,'longitude': lng,'altitude':altitud,'battery':battery,'distanceTarget':distanceTarget,'numberWaypoint':numberWaypoint,'fenceBreach':fence,'failBattery':failBattery, 'token':objToken.getLocalToken(),'flightMode':flightMode,'distanceHome':distanceHome,'numberSatellites':numberSatellites,'hdop':hdop,'fixType':fixType,'stateArmed':stateArmed,'angle':angle,'flagObstacle':flagObstacle}
			r = requests.post(url, data = json.dumps(payload), headers = headers)

	return "Hello, World Virtuxplorer!"


#@@@@@@@@@ FIN DE FUNCIONES DE TELEMETRIA @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@



#@@@@@@@ FUNCION PRINCIPAL QUE RECIBE COMANDOS DESDE VIRTUXPLORER @@@@@@@@@@@@@@@@@@@@@

@app.route('/sendCommand',methods=['POST'])
def index():


	global objConnection
	global objSupervisor
	global objFence
	global objToken
	
	ipServer=str(objConnection.getIpServer())
	portServer=str(objConnection.getPortServer())
	timeInterval=1.0

	if request.json:

		master=mpstate.master()
		stateArmed=mpstate.master().motors_armed()
		line=str(request.json['command'])
		incomingToken = request.json['token']
		print("token",incomingToken)
		print("line",line)
		# Se obiene el token y se guarda

		if(line=="createConnection" and stateArmed==False):

			print("entro a create connection",line)
			idFlight=request.json['idFlight']
			idMultirotor = request.json['idMultirotor']
			url='http://'+ ipServer + ':' + portServer+'/Virtuxplorer/TelemetryWebservice/validateToken'
			#url='https://virtuxplorer.localtunnel.me/Virtuxplorer/TelemetryWebservice/validateToken'
			headers = {'content-type': 'application/json'}
			payload = {'idFlight': idFlight, 'token': incomingToken, 'idMultirotor': idMultirotor}
			r = requests.post(url, data = json.dumps(payload), headers = headers)
			respuesta = str(r.json()['exito'])
			print("La respuesta fue a la validacion del token fue ",respuesta)
			print ("token:",incomingToken)

			if(respuesta == "1"):
				objToken=Token(incomingToken, True)
				objSupervisor=Supervisor()
				objConnection.finishOldFlights()
				objConnection.initializeConnection()
				objVideo=Camera()
				objSupervisor.setFlagCheckConnectionVirtuxplorer(True)
				startTimeFlight()
				save_log("MESSAGE: Initializing connection, the token is valid")
				print "TOKEN_VALIDO"

			else:

				#print respuesta
				print "TOKEN_INVALIDO"
				save_log("WARNING: The connection could not be successfully made, the token is invalid")

	# Que pasa si el token esta mal?

	# Si ya se comprobo que el token es valido y es igual el token que viene al que quedo

		if(objToken.getIsValidToken()==True and (objToken.getLocalToken() == incomingToken)):
			flightMode=str(mpstate.status.flightmode)
			altitud=float(master.field('GLOBAL_POSITION_INT', 'relative_alt',0)* 1.0e-3)
			flagFence=objSupervisor.getFlagFence()
			fenceMinAltitude=objSupervisor.getFenceMinAltitude()
			flagObstacleNear=objSupervisor.getFlagObstacleNear()
			flagControl=objSupervisor.getFlagControl()
			
			if(line=="setFMR" and stateArmed==False):

				heightMin=float(request.json['heightMin'])
				heightMax=request.json['heightMax']
				listCoordinates=request.json['listCoordinatesFence']
				listRectanglesReceived=request.json['listCoordinatesRectangles']                   
				listCommands=request.json['listCoordinatesMission']
				listRally=request.json['listCoordinatesRally']
				objFence=Fence(heightMin,heightMax, listCoordinates,listRectanglesReceived,listCommands, listRally)
				
				
			elif(line=="establishVertical" and flagFence==0 and fenceMinAltitude==0 and  flagObstacleNear==False and flagControl==True and flightMode=="POSHOLD" and stateArmed==True):

				objSupervisor.setFlagControl(False)
				objSupervisor.setChannelModified('',0)
				print "establishVertical"
				line='rc 3 1500'
				mpstate.input_queue.put(line)
				save_log("MESSAGE: Establish throttle")


			elif(line=="establishHorizontal" and flagFence==0 and fenceMinAltitude==0 and flagObstacleNear==False and flagControl==True and flightMode=="POSHOLD" and stateArmed==True):

				objSupervisor.setFlagControl(False)
				objSupervisor.setChannelModified('',0)
				print "establishHorizontal"
				line='rc 1 1500'
				mpstate.input_queue.put(line)
				line='rc 3 1500'
				mpstate.input_queue.put(line)
				line='rc 1 0'
				mpstate.input_queue.put(line)
				save_log("MESSAGE: Establish roll")

			elif(line=="establishForward" and flagFence==0 and fenceMinAltitude==0 and flagObstacleNear==False and flagControl==True and flightMode=="POSHOLD" and stateArmed==True):

				objSupervisor.setFlagControl(False)
				objSupervisor.setChannelModified('',0)
				print "establishForward"
				line='rc 3 1500'
				mpstate.input_queue.put(line)
				line='rc 2 1500'
				mpstate.input_queue.put(line)
				line='rc 2 0'
				mpstate.input_queue.put(line)
				save_log("MESSAGE: Establish pitch")
				#line='rc 3 0'
				#mpstate.input_queue.put(line)


			elif(line=="establishRotate" and flagFence==0 and fenceMinAltitude==0  and flagObstacleNear==False and flagControl==True and flightMode=="POSHOLD" and stateArmed==True):

				objSupervisor.setFlagControl(False)
				objSupervisor.setChannelModified('',0)
				print "establishRotate"
				line='rc 4 1500'
				mpstate.input_queue.put(line)
				line='rc 3 1500'
				mpstate.input_queue.put(line)
				line='rc 4 0'
				mpstate.input_queue.put(line)
				save_log("MESSAGE: Establish yaw")
				#line='rc 3 0'
				#mpstate.input_queue.put(line)


			elif(line=="moveLeft" and flagFence==0 and fenceMinAltitude==0 and flagObstacleNear==False and flagControl==False and flightMode=="POSHOLD" and stateArmed==True):

				print "moveLeft"
				objSupervisor.setChannelModified('rollLeft',1400)
				objSupervisor.setFlagControl(True)
				line='rc 3 1500'
				mpstate.input_queue.put(line)   
				line='rc 1 1400'
				mpstate.input_queue.put(line)
				Timer(3.0,objSupervisor.checkGroundSpeed).start()
				save_log("MESSAGE: Move left")

			elif(line=="moveRight" and flagFence==0 and fenceMinAltitude==0  and flagObstacleNear==False and flagControl==False and flightMode=="POSHOLD" and stateArmed==True):

				print "moveRight"
				objSupervisor.setChannelModified('rollRight',1600)
				objSupervisor.setFlagControl(True)
				line='rc 3 1500'
				mpstate.input_queue.put(line)
				line='rc 1 1600'
				mpstate.input_queue.put(line)
				Timer(3.0,objSupervisor.checkGroundSpeed).start()
				save_log("MESSAGE: Move right")

			elif(line=="moveFront" and flagFence==0 and fenceMinAltitude==0 and flagObstacleNear==False and flagControl==False and flightMode=="POSHOLD" and stateArmed==True):

				print "moveFront"
				objSupervisor.setChannelModified('pitchFront',1400)
				objSupervisor.setFlagControl(True)
				line='rc 3 1500'
				mpstate.input_queue.put(line)
				line='rc 2 1400'
				mpstate.input_queue.put(line)
				Timer(3.0,objSupervisor.checkGroundSpeed).start()
				save_log("MESSAGE: Move front")

			elif(line=="moveBack" and flagFence==0 and fenceMinAltitude==0 and flagObstacleNear==False and flagControl==False and flightMode=="POSHOLD" and stateArmed==True):

				print "moveBack"
				objSupervisor.setChannelModified('pitchBack',1600)
				objSupervisor.setFlagControl(True)
				line='rc 3 1500'
				mpstate.input_queue.put(line)
				line='rc 2 1600'
				mpstate.input_queue.put(line)
				Timer(3.0,objSupervisor.checkGroundSpeed).start()
				save_log("MESSAGE: Move back")

			elif(line=="moveUp" and flagFence==0 and fenceMinAltitude==0 and flagObstacleNear==False and flagControl==False and flightMode=="POSHOLD" and stateArmed==True):

				print "moveUp"
				objSupervisor.setChannelModified('',0)
				objSupervisor.setFlagControl(True)
				line='rc 3 1800'
				mpstate.input_queue.put(line)
				save_log("MESSAGE: Move up")

			elif(line=="moveDown" and flagFence==0 and fenceMinAltitude==0  and  flagObstacleNear==False and flagControl==False and flightMode=="POSHOLD" and stateArmed==True):

				print "moveDown"
				objSupervisor.setChannelModified('',0)
				objSupervisor.setFlagControl(True)                                                                                                  
				line='rc 3 1300'
				mpstate.input_queue.put(line)
				save_log("MESSAGE: Move down")

			elif(line=="moveClock" and flagFence==0 and fenceMinAltitude==0 and flagObstacleNear==False and flagControl==False and flightMode=="POSHOLD" and stateArmed==True):

				print "moveClock"
				objSupervisor.setChannelModified('',0)
				objSupervisor.setFlagControl(True)                                                                                               
				line='rc 3 1500'
				mpstate.input_queue.put(line)   
				line='rc 4 1600'
				mpstate.input_queue.put(line)
				save_log("MESSAGE: Move clock")

			elif(line=="moveCounterClock" and flagFence==0 and fenceMinAltitude==0  and flagObstacleNear==False and flagControl==False and flightMode=="POSHOLD" and stateArmed==True):

				print "moveCounterClock"
				objSupervisor.setChannelModified('',0)
				objSupervisor.setFlagControl(True)                                           
				line='rc 3 1500'
				mpstate.input_queue.put(line)
				line='rc 4 1400'
				mpstate.input_queue.put(line)
				save_log("MESSAGE: Move counter clock")                  

			elif(line=="BACK_HOME" and flagFence==0 and fenceMinAltitude==0 and flagObstacleNear==False and stateArmed==True):

				objSupervisor.setChannelModified('',0)
				objSupervisor.setFlagControl(False)
				objFlight=FlightAction()
				objFlight.backHome()
				save_log("MESSAGE: Mode RTL activated")

			elif(line=="DO_MISSION" and flagFence==0 and fenceMinAltitude==0 and  flagObstacleNear==False):

				objSupervisor.setChannelModified('',0)
				objSupervisor.setFlagControl(False)
				objFlight=FlightAction()
				objFlight.doMission()
		 	   


			elif(line=="STOP_MOVEMENT" and flagFence==0 and fenceMinAltitude==0 and flagObstacleNear==False and flightMode=="POSHOLD" and altitud>objFence.getHeightMin() and stateArmed==True):

				objSupervisor.setChannelModified('',0)
				objSupervisor.setFlagControl(False)
				line='rc 1 1500'
				mpstate.input_queue.put(line)
				line='rc 2 1500'
				mpstate.input_queue.put(line)
				line='rc 3 1500'
				mpstate.input_queue.put(line)
				line='rc 4 1500'
				mpstate.input_queue.put(line)
				line='rc 1 0'
				mpstate.input_queue.put(line)
				line='rc 2 0'
				mpstate.input_queue.put(line)
				line='rc 3 1500'
				mpstate.input_queue.put(line)
				line='rc 4 0'
				mpstate.input_queue.put(line)
				save_log("WARNING: Stop movement")

			elif(line=="MANUAL" and flagFence==0 and fenceMinAltitude==0 and flagObstacleNear==False and flightMode=="AUTO" and stateArmed==True):
				objFlight=FlightAction()
				objFlight.changePoshold()
				save_log("MESSAGE: Manual mode activated")

			elif(line=="TAKEOFF" and flagFence==0 and fenceMinAltitude==0 and flightMode=="GUIDED" and stateArmed==False and flagObstacleNear==False):
				objFlight=FlightAction()
				objFlight.takeoff()
				save_log("MESSAGE: Takeoff received")
			
			elif(line=="MOVE_UP_CAMERA"):
				objCamera=Camera()
				objCamera.moveCameraUp()
				save_log("MESSAGE: Move up camera received")
			
			elif(line=="MOVE_DOWN_CAMERA"):
				save_log("MESSAGE: Move down camera received")
				objCamera=Camera()
				objCamera.moveCameraDown()
				
			elif(line=="CENTER_CAMERA"):
				save_log("MESSAGE: Stop movement camera received")
				objCamera=Camera()
				objCamera.centerCamera()


			else:
				mpstate.input_queue.put(line)

	return "Hello, World Virtuxplorer!"
         
#@@@@@@@@@@@@@@@ FIN DE FUNCION PRINCIPAL  @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@




         
        
# adding all this allows pyinstaller to build a working windows executable
# note that using --hidden-import does not work for these modules
try:
      from multiprocessing import freeze_support
      from pymavlink import mavwp, mavutil
      import matplotlib, HTMLParser
      try:
            import readline
      except ImportError:
            import pyreadline as readline
except Exception:
      pass

if __name__ == '__main__':
      freeze_support()

mensajeTele=""


class MPStatus(object):
    '''hold status information about the mavproxy'''
    def __init__(self):
        self.gps	 = None
        self.msgs = {}
        self.msg_count = {}
        self.counters = {'MasterIn' : [], 'MasterOut' : 0, 'FGearIn' : 0, 'FGearOut' : 0, 'Slave' : 0}
        self.setup_mode = opts.setup
        self.mav_error = 0
        self.altitude = 0
        self.last_distance_announce = 0.0
        self.exit = False
        self.flightmode = 'MAV'
        self.last_mode_announce = 0
        self.last_mode_announced = 'MAV'
        self.logdir = None
        self.last_heartbeat = 0
        self.last_message = 0
        self.heartbeat_error = False
        self.last_apm_msg = None
        self.last_apm_msg_time = 0
        self.highest_msec = 0
        self.have_gps_lock = False
        self.lost_gps_lock = False
        self.last_gps_lock = 0
        self.watch = None
        self.last_streamrate1 = -1
        self.last_streamrate2 = -1
        self.last_seq = 0
        self.armed = False

    def show(self, f, pattern=None):
        '''write status to status.txt'''
        if pattern is None:
            f.write('Counters: ')
            for c in self.counters:
                f.write('%s:%s ' % (c, self.counters[c]))
            f.write('\n')
            f.write('MAV Errors: %u\n' % self.mav_error)
            f.write(str(self.gps)+'\n')
        for m in sorted(self.msgs.keys()):
            if pattern is not None and not fnmatch.fnmatch(str(m).upper(), pattern.upper()):
                continue
            f.write("%u: %s\n" % (self.msg_count[m], str(self.msgs[m])))

    def write(self):
        '''write status to status.txt'''
        f = open('status.txt', mode='w')
        self.show(f)
        f.close()

def say_text(text, priority='important'):
    '''text output - default function for say()'''
    mpstate.console.writeln(text)

def say(text, priority='important'):
    '''text and/or speech output'''
    mpstate.functions.say(text, priority)

def add_input(cmd, immediate=False):
    '''add some command input to be processed'''
    if immediate:
        process_stdin(cmd)
    else:
        mpstate.input_queue.put(cmd)

class MAVFunctions(object):
    '''core functions available in modules'''
    def __init__(self):
        self.process_stdin = add_input
        self.param_set = param_set
        self.get_mav_param = get_mav_param
        self.say = say_text
        # input handler can be overridden by a module
        self.input_handler = None

class MPState(object):
    '''holds state of mavproxy'''
    def __init__(self):
        self.console = textconsole.SimpleConsole()
        self.map = None
        self.map_functions = {}
        self.vehicle_type = None
        self.vehicle_name = None
        from MAVProxy.modules.lib.mp_settings import MPSettings, MPSetting
        self.settings = MPSettings(
            [ MPSetting('link', int, 1, 'Primary Link', tab='Link', range=(0,4), increment=1),
              MPSetting('streamrate', int, 4, 'Stream rate link1', range=(-1,100), increment=1),
              MPSetting('streamrate2', int, 4, 'Stream rate link2', range=(-1,100), increment=1),
              MPSetting('heartbeat', int, 1, 'Heartbeat rate', range=(0,5), increment=1),
              MPSetting('mavfwd', bool, True, 'Allow forwarded control'),
              MPSetting('mavfwd_rate', bool, False, 'Allow forwarded rate control'),
              MPSetting('shownoise', bool, True, 'Show non-MAVLink data'),
              MPSetting('baudrate', int, opts.baudrate, 'baudrate for new links', range=(0,10000000), increment=1),
              MPSetting('rtscts', bool, opts.rtscts, 'enable flow control'),
              MPSetting('select_timeout', float, 0.01, 'select timeout'),

              MPSetting('altreadout', int, 10, 'Altitude Readout',
                        range=(0,100), increment=1, tab='Announcements'),
              MPSetting('distreadout', int, 200, 'Distance Readout', range=(0,10000), increment=1),

              MPSetting('moddebug', int, opts.moddebug, 'Module Debug Level', range=(0,3), increment=1, tab='Debug'),
              MPSetting('compdebug', int, 0, 'Computation Debug Mask', range=(0,3), tab='Debug'),
              MPSetting('flushlogs', bool, False, 'Flush logs on every packet'),
              MPSetting('requireexit', bool, False, 'Require exit command'),
              MPSetting('wpupdates', bool, True, 'Announce waypoint updates'),

              MPSetting('basealt', int, 0, 'Base Altitude', range=(0,30000), increment=1, tab='Altitude'),
              MPSetting('wpalt', int, 100, 'Default WP Altitude', range=(0,10000), increment=1),
              MPSetting('rallyalt', int, 90, 'Default Rally Altitude', range=(0,10000), increment=1),
              MPSetting('terrainalt', str, 'Auto', 'Use terrain altitudes', choice=['Auto','True','False']),
              MPSetting('rally_breakalt', int, 40, 'Default Rally Break Altitude', range=(0,10000), increment=1),
              MPSetting('rally_flags', int, 0, 'Default Rally Flags', range=(0,10000), increment=1),

              MPSetting('source_system', int, 255, 'MAVLink Source system', range=(0,255), increment=1, tab='MAVLink'),
              MPSetting('source_component', int, 0, 'MAVLink Source component', range=(0,255), increment=1),
              MPSetting('target_system', int, 0, 'MAVLink target system', range=(0,255), increment=1),
              MPSetting('target_component', int, 0, 'MAVLink target component', range=(0,255), increment=1),
              MPSetting('state_basedir', str, None, 'base directory for logs and aircraft directories'),
              MPSetting('allow_unsigned', bool, True, 'whether unsigned packets will be accepted'),

              MPSetting('dist_unit', str, 'm', 'distance unit', choice=['m', 'nm', 'miles'], tab='Units'),
              MPSetting('height_unit', str, 'm', 'height unit', choice=['m', 'feet']),
              MPSetting('speed_unit', str, 'm/s', 'height unit', choice=['m/s', 'knots', 'mph']),

              MPSetting('vehicle_name', str, '', 'Vehicle Name', tab='Vehicle'),
            ])

        self.completions = {
            "script"         : ["(FILENAME)"],
            "set"            : ["(SETTING)"],
            "status"         : ["(VARIABLE)"],
            "module"    : ["list",
                           "load (AVAILMODULES)",
                           "<unload|reload> (LOADEDMODULES)"]
            }

        self.status = MPStatus()

        # master mavlink device
        self.mav_master = None

        # mavlink outputs
        self.mav_outputs = []
        self.sysid_outputs = {}

        # SITL output
        self.sitl_output = None

        self.mav_param = mavparm.MAVParmDict()
        self.modules = []
        self.public_modules = {}
        self.functions = MAVFunctions()
        self.select_extra = {}
        self.continue_mode = False
        self.aliases = {}
        import platform
        self.system = platform.system()

    def module(self, name):
        '''Find a public module (most modules are private)'''
        if name in self.public_modules:
            return self.public_modules[name]
        return None

    def master(self):
        '''return the currently chosen mavlink master object'''
        if len(self.mav_master) == 0:
              return None
        if self.settings.link > len(self.mav_master):
            self.settings.link = 1

        # try to use one with no link error
        if not self.mav_master[self.settings.link-1].linkerror:
            return self.mav_master[self.settings.link-1]
        for m in self.mav_master:
            if not m.linkerror:
                return m
        return self.mav_master[self.settings.link-1]


def get_mav_param(param, default=None):
    '''return a EEPROM parameter value'''
    return mpstate.mav_param.get(param, default)

def param_set(name, value, retries=3):
    '''set a parameter'''
    name = name.upper()
    return mpstate.mav_param.mavset(mpstate.master(), name, value, retries=retries)

def cmd_script(args):
    '''run a script'''
    if len(args) < 1:
        print("usage: script <filename>")
        return

    run_script(args[0])

def cmd_set(args):
    '''control mavproxy options'''
    mpstate.settings.command(args)

def cmd_status(args):
    '''show status'''
    if len(args) == 0:
        mpstate.status.show(sys.stdout, pattern=None)
    else:
        for pattern in args:
            mpstate.status.show(sys.stdout, pattern=pattern)

def cmd_setup(args):
    mpstate.status.setup_mode = True
    mpstate.rl.set_prompt("")


def cmd_reset(args):
    print("Resetting master")
    mpstate.master().reset()

def cmd_watch(args):
    '''watch a mavlink packet pattern'''
    if len(args) == 0:
        mpstate.status.watch = None
        return
    mpstate.status.watch = args[0]
    print("Watching %s" % mpstate.status.watch)

def load_module(modname, quiet=False):
    '''load a module'''
    modpaths = ['MAVProxy.modules.mavproxy_%s' % modname, modname]
    for (m,pm) in mpstate.modules:
        if m.name == modname:
            if not quiet:
                print("module %s already loaded" % modname)
            return False
    for modpath in modpaths:
        try:
            m = import_package(modpath)
            reload(m)
            module = m.init(mpstate)
            if isinstance(module, mp_module.MPModule):
                mpstate.modules.append((module, m))
                if not quiet:
                    print("Loaded module %s" % (modname,))
                return True
            else:
                ex = "%s.init did not return a MPModule instance" % modname
                break
        except ImportError as msg:
            ex = msg
            if mpstate.settings.moddebug > 1:
                import traceback
                print(traceback.format_exc())
    print("Failed to load module: %s. Use 'set moddebug 3' in the MAVProxy console to enable traceback" % ex)
    return False

def unload_module(modname):
    '''unload a module'''
    for (m,pm) in mpstate.modules:
        if m.name == modname:
            if hasattr(m, 'unload'):
                m.unload()
            mpstate.modules.remove((m,pm))
            print("Unloaded module %s" % modname)
            return True
    print("Unable to find module %s" % modname)
    return False

def cmd_module(args):
    '''module commands'''
    usage = "usage: module <list|load|reload|unload>"
    if len(args) < 1:
        print(usage)
        return
    if args[0] == "list":
        for (m,pm) in mpstate.modules:
            print("%s: %s" % (m.name, m.description))
    elif args[0] == "load":
        if len(args) < 2:
            print("usage: module load <name>")
            return
        load_module(args[1])
    elif args[0] == "reload":
        if len(args) < 2:
            print("usage: module reload <name>")
            return
        modname = args[1]
        pmodule = None
        for (m,pm) in mpstate.modules:
            if m.name == modname:
                pmodule = pm
        if pmodule is None:
            print("Module %s not loaded" % modname)
            return
        if unload_module(modname):
            import zipimport
            try:
                reload(pmodule)
            except ImportError:
                clear_zipimport_cache()
                reload(pmodule)
            if load_module(modname, quiet=True):
                print("Reloaded module %s" % modname)
    elif args[0] == "unload":
        if len(args) < 2:
            print("usage: module unload <name>")
            return
        modname = os.path.basename(args[1])
        unload_module(modname)
    else:
        print(usage)


def cmd_alias(args):
    '''alias commands'''
    usage = "usage: alias <add|remove|list>"
    if len(args) < 1 or args[0] == "list":
        if len(args) >= 2:
            wildcard = args[1].upper()
        else:
            wildcard = '*'
        for a in sorted(mpstate.aliases.keys()):
            if fnmatch.fnmatch(a.upper(), wildcard):
                print("%-15s : %s" % (a, mpstate.aliases[a]))
    elif args[0] == "add":
        if len(args) < 3:
            print(usage)
            return
        a = args[1]
        mpstate.aliases[a] = ' '.join(args[2:])
    elif args[0] == "remove":
        if len(args) != 2:
            print(usage)
            return
        a = args[1]
        if a in mpstate.aliases:
            mpstate.aliases.pop(a)
        else:
            print("no alias %s" % a)
    else:
        print(usage)
        return


def clear_zipimport_cache():
    """Clear out cached entries from _zip_directory_cache.
    See http://www.digi.com/wiki/developer/index.php/Error_messages"""
    import sys, zipimport
    syspath_backup = list(sys.path)
    zipimport._zip_directory_cache.clear()

    # load back items onto sys.path
    sys.path = syspath_backup
    # add this too: see https://mail.python.org/pipermail/python-list/2005-May/353229.html
    sys.path_importer_cache.clear()

# http://stackoverflow.com/questions/211100/pythons-import-doesnt-work-as-expected
# has info on why this is necessary.

def import_package(name):
    """Given a package name like 'foo.bar.quux', imports the package
    and returns the desired module."""
    import zipimport		
				
    try:
        mod = __import__(name)
    except ImportError:
        clear_zipimport_cache()
        mod = __import__(name)

    components = name.split('.')
    for comp in components[1:]:
        mod = getattr(mod, comp)
    return mod


command_map = {
    'script'  : (cmd_script,   'run a script of MAVProxy commands'),
    'setup'   : (cmd_setup,    'go into setup mode'),
    'reset'   : (cmd_reset,    'reopen the connection to the MAVLink master'),
    'status'  : (cmd_status,   'show status'),
    'set'     : (cmd_set,      'mavproxy settings'),
    'watch'   : (cmd_watch,    'watch a MAVLink pattern'),
    'module'  : (cmd_module,   'module commands'),
    'alias'   : (cmd_alias,    'command aliases')
    }

def shlex_quotes(value):
    '''see http://stackoverflow.com/questions/6868382/python-shlex-split-ignore-single-quotes'''
    lex = shlex.shlex(value)
    lex.quotes = '"'
    lex.whitespace_split = True
    lex.commenters = ''
    return list(lex)

def process_stdin(line):
    '''handle commands from user'''
      
      
              
    if line is None:
        sys.exit(0)

    # allow for modules to override input handling
    if mpstate.functions.input_handler is not None:
          mpstate.functions.input_handler(line)
          return

    line = line.strip()

    if mpstate.status.setup_mode:
        # in setup mode we send strings straight to the master
        if line == '.':
            mpstate.status.setup_mode = False
            mpstate.status.flightmode = "MAV"
            mpstate.rl.set_prompt("MAV> ")
            return
        if line != '+++':
            line += '\r'
        for c in line:
            time.sleep(0.01)
            mpstate.master().write(c)
        return

    if not line:
        return

    args = shlex_quotes(line)
    cmd = args[0]
    
    #cmd=recibido
    while cmd in mpstate.aliases:
        line = mpstate.aliases[cmd]
        args = shlex.split(line) + args[1:]
        cmd = args[0]

    if cmd == 'help':
        k = command_map.keys()
        k.sort()
        for cmd in k:
            (fn, help) = command_map[cmd]
            print("%-15s : %s" % (cmd, help))
        return
    if cmd == 'exit' and mpstate.settings.requireexit:
        mpstate.status.exit = True
        return

    if not cmd in command_map:
        for (m,pm) in mpstate.modules:
            if hasattr(m, 'unknown_command'):
                try:
                    if m.unknown_command(args):
                        return
                except Exception as e:
                    print("ERROR in command: %s" % str(e))
        print("Unknown command '%s'" % line)
        return
      
    (fn, help) = command_map[cmd]
    print "cmd",cmd
    print fn
    try:
      fn(args[1:])

                  
    except Exception as e:
        print("ERROR in command %s: %s" % (args[1:], str(e)))
        if mpstate.settings.moddebug > 1:
            traceback.print_exc()
    
         

def process_master(m):
    '''process packets from the MAVLink master'''
    try:
        s = m.recv(16*1024)
    except Exception:
        time.sleep(0.1)
        return
    # prevent a dead serial port from causing the CPU to spin. The user hitting enter will
    # cause it to try and reconnect
    if len(s) == 0:
        time.sleep(0.1)
        return

    if (mpstate.settings.compdebug & 1) != 0:
        return

    if mpstate.logqueue_raw:
        mpstate.logqueue_raw.put(str(s))

    if mpstate.status.setup_mode:
        if mpstate.system == 'Windows':
           # strip nsh ansi codes
           s = s.replace("\033[K","")
        sys.stdout.write(str(s))
        sys.stdout.flush()
        return

    if m.first_byte and opts.auto_protocol:
        m.auto_mavlink_version(s)
    msgs = m.mav.parse_buffer(s)
    if msgs:
        for msg in msgs:
            sysid = msg.get_srcSystem()
            if sysid in mpstate.sysid_outputs:
                  # the message has been handled by a specialised handler for this system
                  continue
            if getattr(m, '_timestamp', None) is None:
                m.post_message(msg)
            if msg.get_type() == "BAD_DATA":
                if opts.show_errors:
                    mpstate.console.writeln("MAV error: %s" % msg)
                mpstate.status.mav_error += 1



def process_mavlink(slave):
    '''process packets from MAVLink slaves, forwarding to the master'''
    try:
        buf = slave.recv()
    except socket.error:
        return
    try:
        if slave.first_byte and opts.auto_protocol:
            slave.auto_mavlink_version(buf)
        msgs = slave.mav.parse_buffer(buf)
    except mavutil.mavlink.MAVError as e:
        mpstate.console.error("Bad MAVLink slave message from %s: %s" % (slave.address, e.message))
        return
    if msgs is None:
        return
    if mpstate.settings.mavfwd and not mpstate.status.setup_mode:
        for m in msgs:
            if mpstate.status.watch is not None:
                if fnmatch.fnmatch(m.get_type().upper(), mpstate.status.watch.upper()):
                    mpstate.console.writeln('> '+ str(m))
            mpstate.master().write(m.get_msgbuf())
    mpstate.status.counters['Slave'] += 1


def mkdir_p(dir):
    '''like mkdir -p'''
    if not dir:
        return
    if dir.endswith("/"):
        mkdir_p(dir[:-1])
        return
    if os.path.isdir(dir):
        return
    mkdir_p(os.path.dirname(dir))
    os.mkdir(dir)

def log_writer():
    '''log writing thread'''
    while True:
        mpstate.logfile_raw.write(mpstate.logqueue_raw.get())
        while not mpstate.logqueue_raw.empty():
            mpstate.logfile_raw.write(mpstate.logqueue_raw.get())
        while not mpstate.logqueue.empty():
            mpstate.logfile.write(mpstate.logqueue.get())
        if mpstate.settings.flushlogs:
            mpstate.logfile.flush()
            mpstate.logfile_raw.flush()

# If state_basedir is NOT set then paths for logs and aircraft
# directories are relative to mavproxy's cwd
def log_paths():
    '''Returns tuple (logdir, telemetry_log_filepath, raw_telemetry_log_filepath)'''
    if opts.aircraft is not None:
        dirname = ""
        if opts.mission is not None:
            print(opts.mission)
            dirname += "%s/logs/%s/Mission%s" % (opts.aircraft, time.strftime("%Y-%m-%d"), opts.mission)
        else:
            dirname += "%s/logs/%s" % (opts.aircraft, time.strftime("%Y-%m-%d"))
        # dirname is currently relative.  Possibly add state_basedir:
        if mpstate.settings.state_basedir is not None:
            dirname = os.path.join(mpstate.settings.state_basedir,dirname)
        mkdir_p(dirname)
        highest = None
        for i in range(1, 10000):
            fdir = os.path.join(dirname, 'flight%u' % i)
            if not os.path.exists(fdir):
                break
            highest = fdir
        if mpstate.continue_mode and highest is not None:
            fdir = highest
        elif os.path.exists(fdir):
            print("Flight logs full")
            sys.exit(1)
        logname = 'flight.tlog'
        logdir = fdir
    else:
        logname = os.path.basename(opts.logfile)
        dir_path = os.path.dirname(opts.logfile)
        if not os.path.isabs(dir_path) and mpstate.settings.state_basedir is not None:
            dir_path = os.path.join(mpstate.settings.state_basedir,dir_path)

        logdir = dir_path

    mkdir_p(logdir)
    return (logdir,
            os.path.join(logdir, logname),
            os.path.join(logdir, logname + '.raw'))

def open_telemetry_logs(logpath_telem, logpath_telem_raw):
    '''open log files'''
    if opts.append_log or opts.continue_mode:
        mode = 'a'
    else:
        mode = 'w'

    try:
        mpstate.logfile = open(logpath_telem, mode=mode)
        mpstate.logfile_raw = open(logpath_telem_raw, mode=mode)
        print("Log Directory: %s" % mpstate.status.logdir)
        print("Telemetry log: %s" % logpath_telem)

        #make sure there's enough free disk space for the logfile (>200Mb)
        stat = os.statvfs(logpath_telem)
        if stat.f_bfree*stat.f_bsize < 209715200:
            print("ERROR: Not enough free disk space for logfile")
            mpstate.status.exit = True
            return

        # use a separate thread for writing to the logfile to prevent
        # delays during disk writes (important as delays can be long if camera
        # app is running)
        t = threading.Thread(target=log_writer, name='log_writer')
        t.daemon = True
        t.start()
    except Exception as e:
        print("ERROR: opening log file for writing: %s" % e)
        mpstate.status.exit = True
        return

def set_stream_rates():
    '''set mavlink stream rates'''
    if (not msg_period.trigger() and
        mpstate.status.last_streamrate1 == mpstate.settings.streamrate and
        mpstate.status.last_streamrate2 == mpstate.settings.streamrate2):
        return
    mpstate.status.last_streamrate1 = mpstate.settings.streamrate
    mpstate.status.last_streamrate2 = mpstate.settings.streamrate2
    for master in mpstate.mav_master:
        if master.linknum == 0:
            rate = mpstate.settings.streamrate
        else:
            rate = mpstate.settings.streamrate2
        if rate != -1:
            master.mav.request_data_stream_send(mpstate.settings.target_system, mpstate.settings.target_component,
                                                mavutil.mavlink.MAV_DATA_STREAM_ALL,
                                                rate, 1)

def check_link_status():
    '''check status of master links'''
    tnow = time.time()
    if mpstate.status.last_message != 0 and tnow > mpstate.status.last_message + 5:
        say("no link")
        mpstate.status.heartbeat_error = True
    for master in mpstate.mav_master:
        if not master.linkerror and (tnow > master.last_message + 5 or master.portdead):
            say("link %u down" % (master.linknum+1))
            master.linkerror = True

def send_heartbeat(master):
    if master.mavlink10():
        master.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                                  0, 0, 0)
    else:
        MAV_GROUND = 5
        MAV_AUTOPILOT_NONE = 4
        master.mav.heartbeat_send(MAV_GROUND, MAV_AUTOPILOT_NONE)

def periodic_tasks():
    '''run periodic checks'''
    if mpstate.status.setup_mode:
        return

    if (mpstate.settings.compdebug & 2) != 0:
        return

    if mpstate.settings.heartbeat != 0:
        heartbeat_period.frequency = mpstate.settings.heartbeat

    if heartbeat_period.trigger() and mpstate.settings.heartbeat != 0:
        mpstate.status.counters['MasterOut'] += 1
        for master in mpstate.mav_master:
            send_heartbeat(master)

    if heartbeat_check_period.trigger():
        check_link_status()

    set_stream_rates()

    # call optional module idle tasks. These are called at several hundred Hz
    for (m,pm) in mpstate.modules:
        if hasattr(m, 'idle_task'):
            try:
                m.idle_task()
            except Exception as msg:
                if mpstate.settings.moddebug == 1:
                    print(msg)
                elif mpstate.settings.moddebug > 1:
                    exc_type, exc_value, exc_traceback = sys.exc_info()
                    traceback.print_exception(exc_type, exc_value, exc_traceback,
                                              limit=2, file=sys.stdout)

        # also see if the module should be unloaded:
        if m.needs_unloading:
            unload_module(m.name)

def main_loop():
    '''main processing loop'''
    if not mpstate.status.setup_mode and not opts.nowait:
        for master in mpstate.mav_master:
            send_heartbeat(master)
            if master.linknum == 0:
                print("Waiting for heartbeat from %s" % master.address)
                master.wait_heartbeat()
        set_stream_rates()

    while True:
        if mpstate is None or mpstate.status.exit:
            return
        while not mpstate.input_queue.empty():
            line = mpstate.input_queue.get()
            mpstate.input_count += 1
            cmds = line.split(';')
            print("cmds[0]",cmds[0])
            #print("cmds[1]",cmds[1])
            if len(cmds) == 1 and cmds[0] == "":
                  mpstate.empty_input_count += 1
            for c in cmds:
                process_stdin(c)

        for master in mpstate.mav_master:
            if master.fd is None:
                if master.port.inWaiting() > 0:
                    process_master(master)

        periodic_tasks()

        rin = []
        for master in mpstate.mav_master:
            if master.fd is not None and not master.portdead:
                rin.append(master.fd)
        for m in mpstate.mav_outputs:
            rin.append(m.fd)
        for sysid in mpstate.sysid_outputs:
            m = mpstate.sysid_outputs[sysid]
            rin.append(m.fd)
        if rin == []:
            time.sleep(0.0001)
            continue

        for fd in mpstate.select_extra:
            rin.append(fd)
        try:
            (rin, win, xin) = select.select(rin, [], [], mpstate.settings.select_timeout)
        except select.error:
            continue

        if mpstate is None:
            return

        for fd in rin:
            if mpstate is None:
                  return
            for master in mpstate.mav_master:
                  if fd == master.fd:
                        process_master(master)
                        if mpstate is None:
                              return
                        continue
            for m in mpstate.mav_outputs:
                if fd == m.fd:
                    process_mavlink(m)
                    if mpstate is None:
                          return
                    continue

            for sysid in mpstate.sysid_outputs:
                m = mpstate.sysid_outputs[sysid]
                if fd == m.fd:
                    process_mavlink(m)
                    if mpstate is None:
                          return
                    continue

            # this allow modules to register their own file descriptors
            # for the main select loop
            if fd in mpstate.select_extra:
                try:
                    # call the registered read function
                    (fn, args) = mpstate.select_extra[fd]
                    fn(args)
                except Exception as msg:
                    if mpstate.settings.moddebug == 1:
                        print(msg)
                    # on an exception, remove it from the select list
                    mpstate.select_extra.pop(fd)

      
      
def run_script(scriptfile):
    '''run a script file'''
    try:
        f = open(scriptfile, mode='r')
    except Exception:
        return
    mpstate.console.writeln("Running script %s" % scriptfile)
    for line in f:
        line = line.strip()
        if line == "" or line.startswith('#'):
            continue
        if line.startswith('@'):
            line = line[1:]
        else:
            mpstate.console.writeln("-> %s" % line)
        process_stdin(line)
    f.close()

if __name__ == '__main__':
    from optparse import OptionParser
    
    parser = OptionParser("mavproxy.py [options]")

    parser.add_option("--master", dest="master", action='append',
                      metavar="DEVICE[,BAUD]", help="MAVLink master port and optional baud rate",
                      default=[])
    parser.add_option("--out", dest="output", action='append',
                      metavar="DEVICE[,BAUD]", help="MAVLink output port and optional baud rate",
                      default=[])
    parser.add_option("--baudrate", dest="baudrate", type='int',
                      help="default serial baud rate", default=57600)
    parser.add_option("--sitl", dest="sitl",  default=None, help="SITL output port")
    parser.add_option("--streamrate",dest="streamrate", default=4, type='int',
                      help="MAVLink stream rate")
    parser.add_option("--source-system", dest='SOURCE_SYSTEM', type='int',
                      default=255, help='MAVLink source system for this GCS')
    parser.add_option("--source-component", dest='SOURCE_COMPONENT', type='int',
                      default=0, help='MAVLink source component for this GCS')
    parser.add_option("--target-system", dest='TARGET_SYSTEM', type='int',
                      default=0, help='MAVLink target master system')
    parser.add_option("--target-component", dest='TARGET_COMPONENT', type='int',
                      default=0, help='MAVLink target master component')
    parser.add_option("--logfile", dest="logfile", help="MAVLink master logfile",
                      default='mav.tlog')
    parser.add_option("-a", "--append-log", dest="append_log", help="Append to log files",
                      action='store_true', default=False)
    parser.add_option("--quadcopter", dest="quadcopter", help="use quadcopter controls",
                      action='store_true', default=False)
    parser.add_option("--setup", dest="setup", help="start in setup mode",
                      action='store_true', default=False)
    parser.add_option("--nodtr", dest="nodtr", help="disable DTR drop on close",
                      action='store_true', default=False)
    parser.add_option("--show-errors", dest="show_errors", help="show MAVLink error packets",
                      action='store_true', default=False)
    parser.add_option("--speech", dest="speech", help="use text to speech",
                      action='store_true', default=False)
    parser.add_option("--aircraft", dest="aircraft", help="aircraft name", default=None)
    parser.add_option("--cmd", dest="cmd", help="initial commands", default=None, action='append')
    parser.add_option("--console", action='store_true', help="use GUI console")
    parser.add_option("--map", action='store_true', help="load map module")
    parser.add_option(
        '--load-module',
        action='append',
        default=[],
        help='Load the specified module. Can be used multiple times, or with a comma separated list')
    parser.add_option("--mav09", action='store_true', default=False, help="Use MAVLink protocol 0.9")
    parser.add_option("--mav10", action='store_true', default=False, help="Use MAVLink protocol 1.0")
    parser.add_option("--mav20", action='store_true', default=True, help="Use MAVLink protocol 2.0")
    parser.add_option("--auto-protocol", action='store_true', default=False, help="Auto detect MAVLink protocol version")
    parser.add_option("--nowait", action='store_true', default=False, help="don't wait for HEARTBEAT on startup")
    parser.add_option("-c", "--continue", dest='continue_mode', action='store_true', default=False, help="continue logs")
    parser.add_option("--dialect",  default="ardupilotmega", help="MAVLink dialect")
    parser.add_option("--rtscts",  action='store_true', help="enable hardware RTS/CTS flow control")
    parser.add_option("--moddebug",  type=int, help="module debug level", default=0)
    parser.add_option("--mission", dest="mission", help="mission name", default=None)
    parser.add_option("--daemon", action='store_true', help="run in daemon mode, do not start interactive shell")
    parser.add_option("--profile", action='store_true', help="run the Yappi python profiler")
    parser.add_option("--state-basedir", default=None, help="base directory for logs and aircraft directories")
    parser.add_option("--version", action='store_true', help="version information")
    parser.add_option("--default-modules", default="log,signing,wp,rally,fence,param,relay,tuneopt,arm,mode,calibration,rc,auxopt,misc,cmdlong,battery,terrain,output,adsb", help='default module list')

    (opts, args) = parser.parse_args()

    # warn people about ModemManager which interferes badly with APM and Pixhawk
    if os.path.exists("/usr/sbin/ModemManager"):
        print("WARNING: You should uninstall ModemManager as it conflicts with APM and Pixhawk")

    if opts.mav09:
        os.environ['MAVLINK09'] = '1'
    if opts.mav20 and not opts.mav10:
        os.environ['MAVLINK20'] = '1'
    from pymavlink import mavutil, mavparm
    mavutil.set_dialect(opts.dialect)

    #version information
    if opts.version:
        import pkg_resources
        version = pkg_resources.require("mavproxy")[0].version
        print "MAVProxy is a modular ground station using the mavlink protocol"
        print "MAVProxy Version: " + version
        sys.exit(1)

    # global mavproxy state
    mpstate = MPState()
    mpstate.status.exit = False
    mpstate.command_map = command_map
    mpstate.continue_mode = opts.continue_mode
    # queues for logging
    mpstate.logqueue = Queue.Queue()
    mpstate.logqueue_raw = Queue.Queue()


    if opts.speech:
        # start the speech-dispatcher early, so it doesn't inherit any ports from
        # modules/mavutil
        load_module('speech')

    if not opts.master:
        serial_list = mavutil.auto_detect_serial(preferred_list=['*FTDI*',"*Arduino_Mega_2560*", "*3D_Robotics*", "*USB_to_UART*", '*PX4*', '*FMU*'])
        print('Auto-detected serial ports are:')
        for port in serial_list:
              print("%s" % port)

    # container for status information
    mpstate.settings.target_system = opts.TARGET_SYSTEM
    mpstate.settings.target_component = opts.TARGET_COMPONENT

    mpstate.mav_master = []

    mpstate.rl = rline.rline("MAV> ", mpstate)

    def quit_handler(signum = None, frame = None):
        #print 'Signal handler called with signal', signum
        if mpstate.status.exit:
            print 'Clean shutdown impossible, forcing an exit'
            sys.exit(0)
        else:
            mpstate.status.exit = True

    # Listen for kill signals to cleanly shutdown modules
    fatalsignals = [signal.SIGTERM]
    try:
        fatalsignals.append(signal.SIGHUP)
        fatalsignals.append(signal.SIGQUIT)
    except Exception:
        pass
    if opts.daemon: # SIGINT breaks readline parsing - if we are interactive, just let things die
        fatalsignals.append(signal.SIGINT)

    for sig in fatalsignals:
        signal.signal(sig, quit_handler)

    load_module('link', quiet=True)

    mpstate.settings.source_system = opts.SOURCE_SYSTEM
    mpstate.settings.source_component = opts.SOURCE_COMPONENT

    # open master link
    for mdev in opts.master:
        if not mpstate.module('link').link_add(mdev):
            sys.exit(1)

    if not opts.master and len(serial_list) == 1:
          print("Connecting to %s" % serial_list[0])
          mpstate.module('link').link_add(serial_list[0].device)
    elif not opts.master:
          wifi_device = '0.0.0.0:14550'
          mpstate.module('link').link_add(wifi_device)


    # open any mavlink output ports
    for port in opts.output:
        mpstate.mav_outputs.append(mavutil.mavlink_connection(port, baud=int(opts.baudrate), input=False))

    if opts.sitl:
        mpstate.sitl_output = mavutil.mavudp(opts.sitl, input=False)

    mpstate.settings.streamrate = opts.streamrate
    mpstate.settings.streamrate2 = opts.streamrate

    if opts.state_basedir is not None:
        mpstate.settings.state_basedir = opts.state_basedir

    msg_period = mavutil.periodic_event(1.0/15)
    heartbeat_period = mavutil.periodic_event(1)
    heartbeat_check_period = mavutil.periodic_event(0.33)

    mpstate.input_queue = Queue.Queue()
    mpstate.input_count = 0
    mpstate.empty_input_count = 0
    if opts.setup:
        mpstate.rl.set_prompt("")

    # call this early so that logdir is setup based on --aircraft
    (mpstate.status.logdir, logpath_telem, logpath_telem_raw) = log_paths()

    if not opts.setup:
        # some core functionality is in modules
        standard_modules = opts.default_modules.split(',')
        for m in standard_modules:
            load_module(m, quiet=True)

    if opts.console:
        process_stdin('module load console')

    if opts.map:
        process_stdin('module load map')

    for module in opts.load_module:
        modlist = module.split(',')
        for mod in modlist:
            process_stdin('module load %s' % mod)

    if 'HOME' in os.environ and not opts.setup:
        start_script = os.path.join(os.environ['HOME'], ".mavinit.scr")
        if os.path.exists(start_script):
            run_script(start_script)
    if 'LOCALAPPDATA' in os.environ and not opts.setup:
        start_script = os.path.join(os.environ['LOCALAPPDATA'], "MAVProxy", "mavinit.scr")
        if os.path.exists(start_script):
            run_script(start_script)

    if opts.aircraft is not None:
        start_script = os.path.join(opts.aircraft, "mavinit.scr")
        if os.path.exists(start_script):
            run_script(start_script)
        else:
            print("no script %s" % start_script)

    if opts.cmd is not None:
        for cstr in opts.cmd:
            cmds = cstr.split(';')
            for c in cmds:
                process_stdin(c)

    if opts.profile:
        import yappi    # We do the import here so that we won't barf if run normally and yappi not available
        yappi.start()

    # log all packets from the master, for later replay
    open_telemetry_logs(logpath_telem, logpath_telem_raw)
    
    # run main loop as a thread

    mpstate.status.thread = threading.Thread(target=main_loop, name='main_loop')
    mpstate.status.thread.daemon = True
    mpstate.status.thread.start()
        
    objConnection=Connection()
    objConnection.killProcess()
    objConnection.activateMultirotor()
    objToken=Token(False,'')
    line='set shownoise false'
    mpstate.input_queue.put(line)
    line='set heartbeat 0'
    mpstate.input_queue.put(line)
    line=str(objConnection.getOutputAdd())
    #mpstate.input_queue.put("output add "+line)
    
    
    
    ipRasp=objConnection.getIpRaspberry()
    portNumber=objConnection.getPortRaspTelemetry()
    stateDebug=False
   
    if(objConnection.getStateDebug()=="True"):
    	stateDebug=True	
    app.run(host=ipRasp,port=int(portNumber),threaded=True,debug=stateDebug)
    
    
    flog=open("logTelemetryVirtuxplorer.txt","w")
    flog.write("*********************WARNING AND MESSAGES**********************"+"\n")
    flog.close()
    





    


    


