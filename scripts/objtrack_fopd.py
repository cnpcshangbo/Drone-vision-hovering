#remark:obj tracking with proportional control,single axis x and y, w/o vision control
#original image size:640*480
#fopd
#The import file has changed to let it work with DroneKit 2
#The connecting string has changed to fit Intel Aero Drone
import time
import cv2
import math
from dronekit import connect, VehicleMode# #LocationGlobalRelative
from pymavlink import mavutil
import balloon_config
from balloon_video import balloon_video
#from balloon_utils import get_distance_from_pixels, wrap_PI
# from position_vector import PositionVector
# from find_balloon import balloon_finder
# from fake_balloon import balloon_sim
# import pid
# from attitude_history import AttitudeHistory
from web_server import Webserver
import numpy as np


class Objtrack:
     def __init__(self):

        # First get an instance of the API endpoint (the connect via web case will be similar)
        #self.api = local_connect()    #from droneapi.lib-->__init__.py,commented by ljx 

        # Our vehicle (we assume the user is trying to control the first vehicle attached to the GCS)
        #self.vehicle = self.api.get_vehicles()[0]/dev/ttyMFD1, 921600
        self.vehicle = connect('tcp:127.0.0.1:5760', wait_ready=True)
        self.frame=None
        self.webframe=None
        self.webframe_masked=None
        # self.timelast=time.time()

        # lamda=0.928  
        # self.vx_fopi=fo_pi(1.3079,3.4269,4999)
        # self.vy_fopi=fo_pi(1.3079,3.4269,4999)

        # horizontal velocity pid controller.  maximum effect is 10 degree lean
        # xy_p = balloon_config.config.get_float('general','VEL_XY_P',1.0)
        # xy_i = balloon_config.config.get_float('general','VEL_XY_I',0.0)
        # xy_d = balloon_config.config.get_float('general','VEL_XY_D',0.0)
        # xy_imax = balloon_config.config.get_float('general','VEL_XY_IMAX',10.0)
        # self.vel_xy_pid = pid.pid(xy_p, xy_i, xy_d, math.radians(xy_imax))

        # vertical velocity pid controller.  maximum effect is 10 degree lean
        # z_p = balloon_config.config.get_float('general','VEL_Z_P',2.5)
        # z_i = balloon_config.config.get_float('general','VEL_Z_I',0.0)
        # z_d = balloon_config.config.get_float('general','VEL_Z_D',0.0)
        # z_imax = balloon_config.config.get_float('general','VEL_IMAX',10.0)
        # self.vel_z_pid = pid.pid(z_p, z_i, z_d, math.radians(z_imax))

        # velocity controller min and max speed
        # self.vel_speed_min = balloon_config.config.get_float('general','VEL_SPEED_MIN',0.0)#1
        # self.vel_speed_max = balloon_config.config.get_float('general','VEL_SPEED_MAX',1.0)#4
        #print 'self.vel_speed_max: ',self.vel_speed_max
        #self.vel_speed_last = 0.0   # last recorded speed
        # self.vel_accel = balloon_config.config.get_float('general','VEL_ACCEL', 0.5)    # maximum acceleration in m/s/s
        # self.vel_dist_ratio = balloon_config.config.get_float('general','VEL_DIST_RATIO', 0.5)
        
        #self.condition_yaw(0)


 
     def object_find(self,frame):
        object_found = False
        object_x = 0
        object_y = 0
        object_radius = 0
        
        #filter_low =np.array([85,147,118])# for blue
        #filter_high=np.array([113,234,255])
        #filter_low =np.array([3,36,43])
        #filter_high=np.array([25,231,204])
        #filter_low =np.array([0,30,30])
        #filter_high=np.array([14,255,255])
        #filter_low =np.array([80,30,30])
        #filter_high=np.array([200,255,255])
        #filter_low =np.array([87,108,69])
        #filter_high=np.array([171,255,255])

        #filter_low =np.array([150,90,30]) #for red circle
        #filter_high=np.array([230,230,255])
        #points(10am)         302,21,90
        #points(10am)         301,20,90
        #points(10am)         301,20,90
        #filter_low =np.array([250,10,70]) #for pink circle
        #filter_high=np.array([350,50,130])

        # filter_low =np.array([150,20,30]) #for red circle
        # filter_high=np.array([320,230,255])
        # filter_low =np.array([150,0,0]) #for red circle
        # filter_high=np.array([169,255,255]) #5-6pm
        # print 'min: ', self.min, '|type: ', type(self.min)
        #print 'max: ', self.max

        filter_low =np.array([int(self.min),int(self.smin),int(self.vmin)]) #for red circle
        filter_high=np.array([int(self.max),int(self.smax),int(self.vmax)])

        # Convert BGR to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Threshold the HSV image
        mask = cv2.inRange(hsv, filter_low, filter_high)
        self.webframe_masked=mask

        # Erode
        erode_kernel = np.ones((3,3),np.uint8);
        eroded_img = cv2.erode(mask,erode_kernel,iterations = 1)
        
        # dilate
        dilate_kernel = np.ones((3,3),np.uint8);
        dilate_img = cv2.dilate(eroded_img,dilate_kernel,iterations = 1)
        
        # blob detector
        blob_params = cv2.SimpleBlobDetector_Params()
        blob_params.minDistBetweenBlobs = 0
        blob_params.filterByInertia = False
        blob_params.filterByConvexity = False
        # blob_params.filterByColor = True
        blob_params.blobColor = 255
        blob_params.filterByCircularity = False
        blob_params.filterByArea = 1
        blob_params.minArea = 50
        blob_params.maxArea = 120000
        blob_detector = cv2.SimpleBlobDetector(blob_params)
        keypts = blob_detector.detect(dilate_img)
    
        # draw centers of all keypoints in new image
        blob_img = cv2.drawKeypoints(frame, keypts, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        
        cv2.circle(frame,(320,240),10,(255,255,255),1)
        #cv2.putText(frame,'Center: 320,240',(320,240), font, 1,(255,255,255),2)
        #cv2.putText(frame,'Object coordinate x:' + str(kp_max[0]) + "; y=" + str(kp_max[1]),(int(kp_max[0]),int(kp_max[1])), font, 1,(255,255,255),2)

        #adding text to image
        font = cv2.FONT_HERSHEY_SIMPLEX
        #Python:#cv2.putText(img, text, org, fontFace, fontScale, color[, thickness[, lineType[, bottomLeftOrigin]]]) 
        #cv2.putText(frame,'Love you!',(10,30), font, 1,(255,255,255),2)
        #cv2.putText(frame,'100,100',(100,100),font, 0.3,(255,0,0),1)
        # find largest blob
        # print 'len of key pts: ',len(keypts)
        if len(keypts) > 0:
            kp_max = keypts[0]
            for kp in keypts:
                if kp.size > kp_max.size:
                    kp_max = kp
    
            if kp_max.size>5:
               # draw circle around the largest blob
               cv2.circle(frame,(int(kp_max.pt[0]),int(kp_max.pt[1])),int(kp_max.size),(255,255,255),1)
               #cv2.circle(frame,(320,240),10,(255,0,0),6)    
			   
               #cv2.line(img, Point pt1, Point pt2, color[,thickness[,lineType[,shift]]]) --> img
               cv2.line(frame,(320,240),(int(kp_max.pt[0]),int(kp_max.pt[1])),(255,255,255),1)
               
			   #adding text to image
               font = cv2.FONT_HERSHEY_SIMPLEX
               #Python:#cv2.putText(img, text, org, fontFace, fontScale, color[, thickness[, lineType[, bottomLeftOrigin]]]) 
               #cv2.putText(frame,'object',(int(kp_max.pt[0]),int(kp_max.pt[1])), font, 1,(255,255,255),2)
               cv2.putText(frame,'object x=' + str(kp_max.pt[0])[0:5] + "; y=" + str(kp_max.pt[1])[0:5],(int(kp_max.pt[0])-100,int(kp_max.pt[1])+10), font, 0.3,(255,255,255),1)

               # set the balloon location
               object_found = True
               object_x = kp_max.pt[0]
               object_y = kp_max.pt[1]
               object_radius = kp_max.size
               self.frame=frame
               #print('%d'%balloon_x,'%d'%balloon_y)
        # return results

        return object_found, object_x, object_y, object_radius
     def vel_control(self,vel_setx,vel_sety):
          msg =self.vehicle.message_factory.set_position_target_local_ned_encode(
                                                     0,       # time_boot_ms (not used)
                                                     0, 0,    # target system, target component
                                                     mavutil.mavlink.MAV_FRAME_BODY_NED, # frame
                                                     #mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
                                                     0b0000111111000111, # type_mask (only speeds enabled)
                                                     0, 0, 0, # x, y, z positions (not used)
                                                     #velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
                                                     vel_setx,vel_sety,0,#right side direction
                                                     0, 0, 0, # x, y, z acceleration (not used)
                                                     0, 0)    # yaw, yaw_rate (not used)
          # send command to vehicle

          self.vehicle.send_mavlink(msg)
          #self.vehicle.flush()
          # time.sleep(0.1)
     def condition_yaw(self, heading, relative=False):
        if relative:
            is_relative=1 #yaw relative to direction of travel
        else:
            is_relative=0 #yaw is an absolute angle
        # create the CONDITION_YAW command using command_long_encode()
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
            0, #confirmation
            heading,    # param 1, yaw in degrees original 
            0,          # param 2, yaw speed deg/s
            1,          # param 3, direction -1 ccw, 1 cw
            is_relative, # param 4, relative offset 1, absolute angle 0
            0, 0, 0)    # param 5 ~ 7 not used
        # send command to vehicle
        self.vehicle.send_mavlink(msg)
        #self.vehicle.flush()
        #time.sleep(0.1)
		
     def run(self):

        # yawangle=0.0
        # imgnum=0
        # bal_rc1=self.vehicle.parameters['RC1_TRIM']
        # bal_rc2=self.vehicle.parameters['RC2_TRIM']
        RC7MAX=self.vehicle.parameters['RC7_MAX'] #for tuning
        RC7MIN=self.vehicle.parameters['RC7_MIN']  
        RC6MIN=self.vehicle.parameters['RC6_MIN']
        RC6MAX=self.vehicle.parameters['RC6_MAX']
        RC6MID=(RC6MIN+RC6MAX)/2
       # print "RC6MIN=",RC6MIN,",RC6MAX=",RC6MAX,",RC6MID=",RC6MID
        # last_mode=None
        gain=1.0
        last_gain=1.0

        # deltasatx=0
        # deltasaty=0  
        

        try:
            
            web=Webserver(balloon_config.config.parser,(lambda:self.webframe),(lambda:self.webframe_masked))
            
            #Shang uncommented here. 
			#web=Webserver(balloon_config.config.parser,(lambda:self.frame))

            fourcc = cv2.VideoWriter_fourcc(*'XVID')
            video_name='FOPD_%s.avi' % (time.time())
            video_writer = cv2.VideoWriter(video_name,fourcc, 1.0, (640,480))
            
            print "Initialising camera."
            # initialise video capture
            if not video_writer is None:
              print "started recording video to %s" % video_name
            else:
              print "failed to start recording video to %s" % video_name
            camera = balloon_video.get_camera()
            
            log_gname='FOPD_%s.txt' % (time.time())
            gf = open(log_gname, mode='a')#w
            gf.write("Guided mode log DATA %s\n"%(time.localtime()))
            gf.write("time\t RC5VAL\t yaw_angle\t objposx\t objposy\t RC7VAL\t vx\t vy\t gain\t size\t roll\t pitch\t RC6VAL\n") #coeffx\t coeffy\t
            # gf.close
            
 
            cycle_num=0
            coeffx=1
            coeffy=1
            previous_time=0
            yprevious_error=0
            xprevious_error=0
            upx9=0
            upx8=0
            upx7=0
            upx6=0
            upx5=0
            upx4=0
            upx3=0
            upx2=0
            upx1=0
           
            deltay9=0
            deltay8=0
            deltay7=0
            deltay6=0
            deltay5=0
            deltay4=0
            deltay3=0
            deltay2=0
            deltay1=0
            
            upy9=0
            upy8=0
            upy7=0
            upy6=0
            upy5=0
            upy4=0
            upy3=0
            upy2=0
            upy1=0
           
            deltax9=0
            deltax8=0
            deltax7=0
            deltax6=0
            deltax5=0
            deltax4=0
            deltax3=0
            deltax2=0
            deltax1=0

            print "Ready to go!"

            while(True):
                
              #print(self.vehicle)
              if self.vehicle is None:
                #print('0.5')
                self.vehicle=self.api.get_vehicles()[0]
                print('no vehicle')
              #print('1.5')

              # get a frame
              _, frame = camera.read()
              newx,newy=frame.shape[1],frame.shape[0]
              newframe=cv2.resize(frame,(newx,newy))
 
              self.frame=newframe

              self.max=web.my_static.max;
              self.min=web.my_static.min;
              self.smax=web.my_static.smax
              self.smin=web.my_static.smin
              self.vmax=web.my_static.vmax
              self.vmin=web.my_static.vmin
              #self.balloon_found, xpos, ypos, size = balloon_finder.analyse_frame(frame)
              self.object_found,xpos,ypos,size=self.object_find(newframe)
              #self.object_found,xpos,ypos,size=self.object_find_hough(frame)
              #if self.object_found:              
			  
              #adding text to image
              font = cv2.FONT_HERSHEY_SIMPLEX
              #Python:#cv2.putText(img, text, org, fontFace, fontScale, color[, thickness[, lineType[,bottomLeftOrigin]]]) 
              cv2.putText(newframe,'Object Found? ' + str(self.object_found)[0],(10,20), font, 0.3,(255,255,255),1)
              cv2.putText(newframe,'Mode: ' + self.vehicle.mode.name,(10,40), font, 0.3,(255,255,255),1)
              cv2.putText(newframe,'Time: ' + str(time.time()),(10,60), font, 0.3,(255,255,255),1)
              cv2.putText(newframe,'Alt: ' + str(self.vehicle.location.global_relative_frame),(10,100), font, 0.3,(255,255,255),1)
              cv2.putText(newframe,'RC5: ' + str(self.vehicle.channels['5'])+'  6: '+ str(self.vehicle.channels['5'])+'  7: '+ str(self.vehicle.channels['7']),(10,120), font, 0.3,(255,255,255),1)
              cv2.putText(newframe,'max: ' + str(self.max),(10,240), font, 0.3,(255,255,255),1)
              #print 'New video frame. Time: %s' % (time.time()) 
              #print xpos
              print self.vehicle.channels['5']
              if self.vehicle.channels['5'] is not None:
                 RC5VAL=float(self.vehicle.channels['5']) 
              else:
                 RC5VAL=1400              
              if self.vehicle.channels['6'] is not None:                 
                 RC6VAL=float(self.vehicle.channels['6'])
              else:
                 RC6VAL=1400              
              if self.vehicle.channels['7'] is not None:    
                 RC7VAL=self.vehicle.channels['7']
              else:
                 RC7VAL=1400              
              if RC7VAL==0 or RC7MAX is None:
                 gain=last_gain
              #else:
              #   gain=7.0*(RC7VAL-RC7MIN)/(RC7MAX-RC7MIN)
              
              if RC7VAL>=1099 and RC7VAL<=1150:
                 gain=0
              elif RC7VAL>=1150 and RC7VAL<=1200:
                 gain=0.05
              elif RC7VAL>=1200 and RC7VAL<=1250:
                 gain=0.1
              elif RC7VAL>=1250 and RC7VAL<=1300:
                 gain=0.2
              elif RC7VAL>=1300 and RC7VAL<=1350:
                 gain=0.3
              elif RC7VAL>=1350 and RC7VAL<=1400:
                 gain=0.4
              elif RC7VAL>=1400 and RC7VAL<=1450:
                 gain=0.5
              elif RC7VAL>=1450 and RC7VAL<=1500:
                 gain=0.6
              elif RC7VAL>=1500 and RC7VAL<=1550:
                 gain=0.7
              elif RC7VAL>=1550 and RC7VAL<=1600:
                 gain=0.8	 
              elif RC7VAL>=1600 and RC7VAL<=1650:
                 gain=0.9
              elif RC7VAL>=1650:
                 gain=1.0
              last_gain=gain
              cv2.putText(newframe,'Gain: ' + str(gain),(10,140), font, 0.3,(255,255,255),1)
              cv2.putText(newframe,'Roll: ' + str(self.vehicle.attitude.roll),(10,160), font, 0.3,(255,255,255),1)
              cv2.putText(newframe,'Pitch: ' + str(self.vehicle.attitude.pitch),(10,180), font, 0.3,(255,255,255),1)
              cv2.putText(newframe,'Battery: ' + str(self.vehicle.battery),(10,200), font, 0.3,(255,255,255),1)
              cv2.putText(newframe,'[vx, vy, vz ](in m/s): ' + str(self.vehicle.velocity),(10,220), font, 0.3,(255,255,255),1)

              #gain1=0.05*gain
              #gain=gain1
              #print 'gain is:',gain,'RC7=',RC7VAL

              #RC6VAL=1
              #if RC6VAL==1:
              #if RC6VAL<RC6MID:#vision-controlled
              if 1:
                           
                 #print 'vision-controlled.'
				 
                 yaw_origin=self.vehicle.attitude.yaw
                 roll_origin=self.vehicle.attitude.roll
                 pitch_origin=self.vehicle.attitude.pitch
                 #attitude_origin=self.vehicle.attitude
                 # print "Heading: %s" % self.vehicle.heading
                 cv2.line(newframe,(320,240),(int(320+100*math.sin(-yaw_origin)),int(240-100*math.cos(-yaw_origin))),(255,255,255),1)
                 cv2.putText(newframe,"N: " + str(math.degrees(yaw_origin))[0:5],(int(320+100*math.sin(-yaw_origin)),int(240-100*math.cos(-yaw_origin))),font,0.3,(255,255,255),1)
                 #print 'yaw_origin=', yaw_origin
                 if self.vehicle.mode.name!="LOITER":#visual-guided
                 #if 1:#visual-guided
                    '''
                    if last_mode!="GUIDED":
                       self.condition_yaw(0)
                    last_mode="GUIDED"
                    '''
                    #print "---------------New period-----------------"

                    #print 'vehicle.mode = Guided. Gain =',gain
					
                    if self.object_found: 
                    #if 1: 
                       # print('object found')

                       deltax=xpos-320  
					   #cv2.line(frame,(320,240),320)
                       deltay=240-ypos
                       #print 'yaw-origin: deltaxpos  deltaypos ', yaw_origin,deltax,deltay
                       #print 'roll, pitch=', roll_origin,pitch_origin
					   
                       #Kd=0.013;
                       
                       #dt=time.time()-previous_time #get the sampling time
                       #if previous_time==0:
                       #     dt=0.58
                       
                       #previous_time=time.time()
                       
                      
                       upx=0.6708*upx1 + 3.159*upx2 - 1.998*upx3 - 3.618*upx4+ 2.099*upx5 + 1.753*upx6 - 0.882*upx7 - 0.2949*upx8+ 0.1106*upx9+5.268*deltay  - 8.506*deltay1 - 8.5*deltay2 + 18.26*deltay3 + 1.801*deltay4- 12.23*deltay5 + 2.027*deltay6 + 2.468*deltay7 - 0.5658*deltay8- 0.03059*deltay9
                       
                       if upx>gain:
                            upx=gain
                       elif upx<-gain:
                            upx=-gain
                       
                       upx9=upx8
                       upx8=upx7
                       upx7=upx6
                       upx6=upx5
                       upx5=upx4
                       upx4=upx3
                       upx3=upx2
                       upx2=upx1
                       upx1=upx
                       
                       deltay9=deltay8
                       deltay8=deltay7
                       deltay7=deltay6
                       deltay6=deltay5
                       deltay5=deltay4
                       deltay4=deltay3
                       deltay3=deltay2
                       deltay2=deltay1
                       deltay1=deltay
                        
                       #print "dt:",dt

                       
                       
                       upy=0.6708*upy1 + 3.159*upy2 - 1.998*upy3 - 3.618*upy4+ 2.099*upy5 + 1.753*upy6 - 0.882*upy7 - 0.2949*upy8+ 0.1106*upy9+5.268*deltax  - 8.506*deltax1 - 8.5*deltax2 + 18.26*deltax3 + 1.801*deltax4- 12.23*deltax5 + 2.027*deltax6 + 2.468*deltax7 - 0.5658*deltax8- 0.03059*deltax9
                            
                       if upy>gain:
                            upy=gain
                       elif upy<-gain:
                            upy=-gain


                       upy9=upy8
                       upy8=upy7
                       upy7=upy6
                       upy6=upy5
                       upy5=upy4
                       upy4=upy3
                       upy3=upy2
                       upy2=upy1
                       upy1=upy
                       
                       deltax9=deltax8
                       deltax8=deltax7
                       deltax7=deltax6
                       deltax6=deltax5
                       deltax5=deltax4
                       deltax4=deltax3
                       deltax3=deltax2
                       deltax2=deltax1
                       deltax1=deltax
                       
                       print 'upx, upy=', upx,upy
                       
                         
                       
                       
                       self.vel_control(upx,upy)
                       cv2.putText(newframe,'upx(N): ' + str(upx) + ', upy(E): ' + str(upy),(10,80), font, 0.3,(255,255,255),1)
                       # time.sleep(0.1)
                 
                       #deltatime=time.time()-self.timelast
                       # gf = open(log_gname, mode='a')
                       gf.write("%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n"%(time.time(),RC5VAL,yaw_origin,xpos,ypos,RC7VAL,upx,upy,gain,size,roll_origin,pitch_origin,RC6VAL))
                       # gf.close()

                    else:#not-found
                         print "Guided mode,object not found. Gain = %f; Time = %f" % (gain,time.time())
                         # self.vel_control(0,0)
                         #time.sleep(0.5)

                         # RC6VAL=float(self.vehicle.channels['6'])
                         #deltatime=time.time()-self.timelast
                         # gf = open(log_gname, mode='a')
                         gf.write("%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n"%(time.time(),0,0,-566,-566,0,0,0,0,size,roll_origin,pitch_origin,RC6VAL))
                         # gf.close()

                 else:#non-guided
                        
                        print "Vision control but non-guided. Yaw=", math.degrees(yaw_origin), ", Time=", time.time()

                        # RC6VAL=float(self.vehicle.channels['6'])
                        #deltatime=time.time()-self.timelast
                        # gf = open(log_gname, mode='a')
                        # gf.write("%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n"%(time.time(),0,0,-566,-566,0,0,0,0,size,roll_origin,pitch_origin,RC6VAL))
                        # gf.close()
                        
                        # if last_mode=="GUIDED":
                            # self.vel_control(0,0)
                            # last_mode=self.vehicle.mode.name
                            # time.sleep(0.5)

                                               
 
              # else:#non-vision-controlled
                   
                   # print "non-vision-controlled,landing......"

                   #deltatime=time.time()-self.timelast
                   # gf = open(log_gname, mode='a')
                   # gf.write("%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n"%(time.time(),yaw_origin,angle_yaw,xpos,ypos,error_total,upx,upy,gain,size,RC6VAL,RC6VAL,RC6VAL))
                   # gf.close()
              '''
                   while(self.vehicle.mode.name=="GUIDED"):

                             self.vehicle.mode.name="LAND"
                             self.vehicle.mode = VehicleMode("LAND")
                             self.vehicle.flush()
                             time.sleep(1)
                   while(self.vehicle.armed is True):
                             self.vehicle.armed=False
                             self.vehicle.flush()
                             time.sleep(1)
              '''
              #if RC6VAL<RC6MID:
              video_writer.write(newframe)
                #print 'Video recorded.'
                #print 'RC6VAL:',RC6VAL
                #print 'RC6MID:',RC6MID
              self.webframe=newframe
      
              #time.sleep(1)  
        # handle interrupts
        except KeyboardInterrupt:
            print 'interrupted, exiting'

        # exit and close web server
        print "exiting..."
        gf.close()
        web.close()
        #Shang uncommented here.

        video_writer.release()  
        bus.exit()



strat = Objtrack()
strat.run()
