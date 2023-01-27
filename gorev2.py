from dronekit import connect, VehicleMode,LocationGlobalRelative
import time
import math
import numpy as np
import cv2
from pymavlink import mavutil
connection_string="127.0.0.1:14550"

iha=connect(connection_string,wait_ready=True,timeout=100)
def arm_ol_ve_yuksel(hedef_yukseklik):
	while iha.is_armable==False:
		print("Arm ici gerekli sartlar saglanamadi.")
		time.sleep(1)
	print("Iha su anda armedilebilir")
	
	iha.mode=VehicleMode("GUIDED")
	while iha.mode=='GUIDED':
		print('Guided moduna gecis yapiliyor')
		time.sleep(1.5)

	print("Guided moduna gecis yapildi")
	iha.armed=True
	while iha.armed is False:
		print("Arm icin bekleniliyor")
		time.sleep(1)

	print("Ihamiz arm olmustur")
	
	iha.simple_takeoff(hedef_yukseklik)
	while iha.location.global_relative_frame.alt<=hedef_yukseklik*0.94:
		print("Su anki yukseklik{}".format(iha.location.global_relative_frame.alt))
		time.sleep(0.5)
	print("Takeoff gerceklesti")
def get_distance_metres(right, left):
    rlat, rlon = right
    llat, llon= left
    ralt=right
    lalt=right
    dlat = llat - rlat
    dlong = llon - rlon
    dlat = abs(dlat)
    dlong = abs(dlong)
    dlat = ddToMeter(dlat)
    dlong = ddToMeter(dlong)
    return math.sqrt((dlat*dlat) + (dlong*dlong))
def pozisyon(posx, posy,yaw_rate,posz, iha):
	msg = iha.message_factory.set_position_target_local_ned_encode(
          0,
          0, 0,
          mavutil.mavlink.MAV_FRAME_LOCAL_NED, 
          0b0000011111111000,
          posx, posy, posz, #pozisyonlar(metre)
          0, 0, 0,#hizlar(metre/s)
          0, 0, 0,#akselarasyon(fonksiyonsuz)
          0, math.radians(yaw_rate))#yaw,yaw_rate(rad,rad/s)

        iha.send_mavlink(msg)
def ddToMeter(decimalDegree):
    return (1.1132 * decimalDegree) / 0.00001
def konumdogrulama(iha,konum, kontrol):
        while True: 
            mesafe = get_distance_metres((iha.location.global_relative_frame.lat,iha.location.global_relative_frame.lon), konum)
            if mesafe < kontrol:
                return True
            print ("Kalan mesafe : {}".format(mesafe))
            return False
def velocity(velocity_x, velocity_y,yaw_rate,velocity_z, iha):
	msg = iha.message_factory.set_position_target_local_ned_encode(
          0,
          0, 0,
          mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, 
          0b0000011111000111,
          0, 0, 0, 
          velocity_x, velocity_y, velocity_z,
          0, 0, 0,
          0, math.radians(yaw_rate))

        iha.send_mavlink(msg)
konumevx=(iha.location.global_relative_frame.lat)
konumevy=(iha.location.global_relative_frame.lon)
konum1=(  -35.36263329 ,149.16479675)
konum2=( -35.36265578,149.16413582)
konum3=( -35.36331829 ,149.16420471)
konum4=( -35.36316106 ,149.16483809)
lokasyon1 = LocationGlobalRelative(  -35.36263329 ,149.16479675,10 )
lokasyon2 = LocationGlobalRelative( -35.36265578, 149.16413582,10 )
lokasyon3 = LocationGlobalRelative( -35.36331829 ,149.16420471,10)
lokasyon4 = LocationGlobalRelative( -35.36316106 ,149.16483809,10)
lokasyonev=LocationGlobalRelative(konumevx,konumevy,10)
lokasyonmavihavuz=LocationGlobalRelative(-35.36292524,149.16505839,10)
lokasyonmavihavuzinis=LocationGlobalRelative(-35.36292524,149.16505839,2)
konummavihavuz=(-35.36292524,149.16505839)
time.sleep(1)
arm_ol_ve_yuksel(10)
iha.groundspeed=8
print("hiz 8 m/s")
iha.simple_goto(lokasyon1)
while not konumdogrulama(iha,konum1,1):
		iha.simple_goto(lokasyon1)
		print("lokasyon 1e gidiliyor")
		print("hiz 8 m/s")
		time.sleep(1)
		print("Su anki koordinat{}, {} ".format(iha.location.global_relative_frame.lat,iha.location.global_relative_frame.lon))
print("lokasyon1e ulasildi")
time.sleep(1)
iha.groundspeed=8
iha.simple_goto(lokasyon2)
print("hiz 8 m/s")
while not konumdogrulama(iha,konum2,1):
		iha.simple_goto(lokasyon2)
		print("lokasyon 2e gidiliyor")
		print("hiz 6 m/s")
		time.sleep(1)
		print("Su anki koordinat{}, {} ".format(iha.location.global_relative_frame.lat,iha.location.global_relative_frame.lon))
print("lokasyon2e ulasildi")
iha.simple_goto(lokasyon3)
iha.groundspeed=2
print("hiz 2 m/s")
camera = cv2.VideoCapture(0)
time.sleep(3)
print("goruntu isleme devrede")
while True:
	belirli = 0
	ret, image = camera.read()
	ters = cv2.flip(image, 0)
	hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
	lower = np.array([160, 120, 55])
	upper = np.array([179, 255, 255])
	mask = cv2.inRange(hsv, lower, upper)
	_, contours,_ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
	areas= [cv2.contourArea(c) for c in contours]
	print(len(contours))
	if len(contours) == 0:
	   pass
	else:
	   max_index = np.argmax(areas)
	   cnt = contours[max_index]
	   c = contours[0]
	   area = cv2.contourArea(c)
	   x, y, w, h = cv2.boundingRect(cnt)
	   (x, y), radius = cv2.minEnclosingCircle(cnt)
	   center = (int(x), int(y))
	   radius = int(radius)
	   solust=(int(x+(h)-w/2), int(y+w/2))
	   sagalt=(int(x-(h)+w/2), int(y-w/2))
	   cv2.circle(image,center,radius,(0, 255, 0), 2)
	   cv2.rectangle(image, solust,sagalt, (0, 255, 0), 2)
	   center2 = (325, 225)
	   cv2.rectangle(image,(300,200),(345,245), (0, 255, 0), 2)
	   cv2.line(image, center, center2, (0, 0, 255), 2)
	   cv2.putText(image, "kirmizi cember", (int(x-30), int(y-40)),
		cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 0), 2)
	   cv2.putText(
		image, "merkez", (center2[0]+10, center2[1]+10), cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 0), 2)
	   print(center)
	   if int(x)>0:
		print("kirmizi renk goruldu")
	   if 300 <= int(x) | int(x) <= 345:
		if 200 <= int(y) | int(y) <= 245:
		    cv2.putText(image, "Belirlendi", ((20, 90)),
			      cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2)
		    belirli=1
		    kirmizihavuzx=(iha.location.global_relative_frame.lat)
		    kirmizihavuzy=(iha.location.global_relative_frame.lon)
		    print("konum alindi")
		    break
		else :
		   if int(y) >= 245:
			velocity(-0.40,0,0,0,iha)
			print("geri gidiliyor")
		   if int(y) <= 200:
			velocity(0.40,0,0,0,iha)
			print("ileri gidiliyor")
	   else :
		   if int(x) >= 345:
			velocity(0,0.40,0,0,iha)
			print("saga gidiliyor")
		   if int(x) <= 300:
			velocity(0,-0.40,0,0,iha)
			print("sola gidiliyor")
	   
	cv2.imshow("deneme", image)
	if cv2.waitKey(25) & 0xFF == ord("q"):
  	    break
camera.release()
cv2.destroyAllWindows()
iha.groundspeed=8
iha.simple_goto(lokasyon3)
print("hiz 8 m/s")
while not konumdogrulama(iha,konum3,1):
		iha.simple_goto(lokasyon3)
		print("lokasyon 3e gidiliyor, hiz 6 m/s")
		time.sleep(1)
		print("Su anki koordinat{}, {} ".format(iha.location.global_relative_frame.lat,iha.location.global_relative_frame.lon))
print("lokasyon3e ulasildi")
time.sleep(1)
iha.groundspeed=8
print("hiz 8 m/s")
iha.simple_goto(lokasyon4)
while not konumdogrulama(iha,konum4,1):
	iha.simple_goto(lokasyon4)
	print("lokasyo4e gidiliyor,hiz 6m/s")
	time.sleep(1)
	print("Su anki koordinat{} ,{} ".format(iha.location.global_relative_frame.lat,iha.location.global_relative_frame.lon))
print("lokasyon4e ulasildi")
iha.simple_goto(lokasyonmavihavuz)
while not konumdogrulama(iha,konummavihavuz,1.2):
	iha.simple_goto(lokasyonmavihavuz)
	print("mavi havuza gidiliyor")
	time.sleep(1)
	print("Su anki koordinat{} ,{} ".format(iha.location.global_relative_frame.lat,iha.location.global_relative_frame.lon))
print("mavi havuza ulasildi")
iha.simple_goto(lokasyonmavihavuzinis)
time.sleep(5)
print("motor")
iha.simple_goto(lokasyonmavihavuz)
time.sleep(5)
if belirli==1:
	hedef=LocationGlobalRelative(kirmizihavuzx,kirmizihavuzy,10)
	hedef2=LocationGlobalRelative(kirmizihavuzx,kirmizihavuzy,4)
	iha.simple_goto(hedef)
	while not konumdogrulama(iha,(kirmizihavuzx,kirmizihavuzy),1.2):
		iha.simple_goto(hedef)
		print("kirmizi havuza gidiliyor")
		time.sleep(1)
		print("Su anki koordinat{} ,{} ".format(iha.location.global_relative_frame.lat,iha.location.global_relative_frame.lon))
	print("kirmizi havuza ulasildi")
	camera = cv2.VideoCapture(0)
	time.sleep(3)
	while True:
		ret, image = camera.read()
		ters = cv2.flip(image, 0)
		hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
		lower = np.array([160, 120, 55])
		upper = np.array([179, 255, 255])
		mask = cv2.inRange(hsv, lower, upper)
		_, contours,_ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
		areas= [cv2.contourArea(c) for c in contours]
		print(len(contours))
		if len(contours) == 0:
		   pass
		else:
		   max_index = np.argmax(areas)
		   cnt = contours[max_index]
		   c = contours[0]
		   area = cv2.contourArea(c)
		   x, y, w, h = cv2.boundingRect(cnt)
		   (x, y), radius = cv2.minEnclosingCircle(cnt)
		   center = (int(x), int(y))
		   radius = int(radius)
		   solust=(int(x+(h)-w/2), int(y+w/2))
		   sagalt=(int(x-(h)+w/2), int(y-w/2))
		   cv2.circle(image,center,radius,(0, 255, 0), 2)
		   cv2.rectangle(image, solust,sagalt, (0, 255, 0), 2)
		   center2 = (325, 225)
		   cv2.rectangle(image,(300,200),(345,245), (0, 255, 0), 2)
		   cv2.line(image, center, center2, (0, 0, 255), 2)
		   cv2.putText(image, "kirmizi cember", (int(x-30), int(y-40)),
			cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 0), 2)
		   cv2.putText(
			image, "merkez", (center2[0]+10, center2[1]+10), cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 0), 2)
		   print(center)
		   if int(x)>0:
			print("kirmizi renk goruldu")
		   if 300 <= int(x) | int(x) <= 345:
			if 200 <= int(y) | int(y) <= 245:
			    cv2.putText(image, "Belirlendi", ((20, 90)),
				      cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2)
			    print("inis yapiliyor")
			    kirmizihavuzx2=(iha.location.global_relative_frame.lat)
	  		    kirmizihavuzy2=(iha.location.global_relative_frame.lon)
			    hedefkirmizi=LocationGlobalRelative(kirmizihavuzx2,kirmizihavuzy2,4)
			    hedefkirmizikalk=LocationGlobalRelative(kirmizihavuzx2,kirmizihavuzy2,10)
			    iha.simple_goto(hedefkirmizi)
			    time.sleep(5)
			    print("servo")
			    iha.simple_goto(hedefkirmizikalk)
			    time.sleep(2)
			    break
			else :
			   if int(y) >= 245:
				velocity(-0.4,0,0,0,iha)
				print("geri gidiliyor")
			   if int(y) <= 200:
				velocity(0.4,0,0,0,iha)
				print("ileri gidiliyor")
		   else :
			   if int(x) >= 345:
				velocity(0,0.4,0,0,iha)
				print("saga gidiliyor")
			   if int(x) <= 300:
				velocity(0,-0.4,0,0,iha)
				print("sola gidiliyor")  
		cv2.imshow("deneme", image)
		if cv2.waitKey(25) & 0xFF == ord("q"):
		    break
	camera.release()
	cv2.destroyAllWindows()
else :
	print("konum alinamadi eve donuluyor")
iha.groundspeed=8
iha.simple_goto(lokasyon3)
while not konumdogrulama(iha,konum3,1):
		iha.simple_goto(lokasyon3)
		print("lokasyon 3e gidiliyor")
		time.sleep(1)
		print("Su anki koordinat{}, {} ".format(iha.location.global_relative_frame.lat,iha.location.global_relative_frame.lon))
print("lokasyon3e ulasildi")
iha.simple_goto(lokasyon4)
while not konumdogrulama(iha,konum4,1):
		iha.simple_goto(lokasyon4)
		print("lokasyon 4e gidiliyor")
		time.sleep(1)
		print("Su anki koordinat{}, {} ".format(iha.location.global_relative_frame.lat,iha.location.global_relative_frame.lon))
print("lokasyon4e ulasildi")
iha.simple_goto(lokasyonev)
while not konumdogrulama(iha,(konumevx,konumevy),1):
	iha.simple_goto(lokasyonev)
	print("eve gidiliyor")
	time.sleep(1)
	print("Su anki koordinat{} ,{} ".format(iha.location.global_relative_frame.lat,iha.location.global_relative_frame.lon))
print("eve ulasildi")
time.sleep(5)
print("inis yapiliyor")
iha.mode = VehicleMode("LAND")
#deneme deneme deneme





















