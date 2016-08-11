#remark:obj tracking with proportional control,single axis x and y, w/o vision control
#original image size:640*480
#fopi
import time
import cv2
import math
from droneapi.lib import VehicleMode, Location
from pymavlink import mavutil
import balloon_config
from balloon_video import balloon_video
from balloon_utils import get_distance_from_pixels, wrap_PI
from position_vector import PositionVector
from find_balloon import balloon_finder
from fake_balloon import balloon_sim
import pid
from attitude_history import AttitudeHistory
from web_server import Webserver
import numpy as np

class fo_pi(object):

    def __init__(self, initial_p=0, initial_i=0,initial_FO_L=4999):
        # default config file
        self.kp = initial_p
        self.ki = initial_i
        self.FO_L=initial_FO_L #calculation length
        self.last_error = None
        self.last_update = time.time()

        self.uk=[0 for i in range(self.FO_L)]
        self.uk2=[0 for i in range(self.FO_L)]

        self.err=[0 for i in range(self.FO_L)]
        self.errsat=[0 for i in range(self.FO_L)]

        self.uk[0]=0
        self.errsat[0]=0

        self.index=0


    # __str__ - print position vector as string
    def __str__(self):
        return "P:%s,I:%s,D:%s,Alpha:%s" % (self.kp, self.ki, self.kd, self.alpha)

    # get_dt - returns time difference since last get_dt call
    def get_dt(self, max_dt):
        now = time.time()
        time_diff = now - self.last_update
        self.last_update = now
        if time_diff > max_dt:
            return 0.0
        else:
            return time_diff

    # get_p - return p term
    def get_p(self, error):
        return self.kp * error

    # get_fopi - return fopi term
    def get_fopi(self, error, dt):
           #ssmpling period:Ts=0.25s
           #ssmpling period:Ts=0.25s
           '''
           #2nd order system
           a1=-0.05453
           a2=0.7413
           a3=0.02683
           b0=14.64
           b1=-32.18
           b2=26.24
           b3=-8.019
 
           self.err[self.index]=error
           if self.index==0:
              self.uk[self.index]=b0*self.err[self.index]   
              
           elif self.index==1:
              self.uk[self.index]=a1*self.uk[self.index-1]+b0*self.err[self.index]+b1*self.err[self.index-1]
              
           elif self.index==2:
              
              self.uk[self.index]=a1*self.uk[self.index-1]+a2*self.uk[self.index-2]+b0*self.err[self.index]+b1*self.err[self.index-1]+b2*self.err[self.index-2]
           elif self.index>=3:
  
              self.uk[self.index]=a1*self.uk[self.index-1]+a2*self.uk[self.index-2]+a3*self.uk[self.index-3]+b0*self.err[self.index]+b1*self.err[self.index-1]+b2*self.err[self.index-2]+b3*self.err[self.index-3]
              ret=self.uk[self.index]
           '''
           #1st order system
           
           a1=-2.208
           a2=1.504
           a3=-0.2959
           b0=0.244
           b1=-0.275
           b2=0.0355
           b3=0.01417
 
           self.err[self.index]=error
           kp=self.kp
           ki=self.ki
           kt=ki
           if self.index==0:
              self.uk2[self.index]=b0*kp*ki*self.err[self.index]
           
           elif self.index==1:
              self.uk2[self.index]=-a1*self.uk[self.index-1]+b0*kp*ki*self.err[self.index]+b1*kp*ki*self.err[self.index-1]-kt*b0*self.errsat[self.index]-kt*b1*self.errsat[self.index-1]

           elif self.index==2:
              self.uk2[self.index]=-a1*self.uk[self.index-1]-a2*self.uk[self.index-2]+b0*kp*ki*self.err[self.index]+b1*kp*ki*self.err[self.index-1]+b2*kp*ki*self.err[self.index-2]-kt*b0*self.errsat[self.index]-kt*b1*self.errsat[self.index-1]-kt*b2*self.errsat[self.index-2]
              
           elif self.index>=3:
              self.uk2[self.index]=-a1*self.uk[self.index-1]-a2*self.uk[self.index-2]-a3*self.uk[self.index-3]+b0*kp*ki*self.err[self.index]+b1*kp*ki*self.err[self.index-1]+b2*kp*ki*self.err[self.index-2]+b3*kp*ki*self.err[self.index-3]-kt*b0*self.errsat[self.index]-kt*b1*self.errsat[self.index-1]-kt*b2*self.errsat[self.index-2]-kt*b3*self.errsat[self.index-3]
           self.uk[self.index]=kp*self.err[self.index]+self.uk2[self.index]   
                      
           u0=self.uk[self.index]
           uf=u0
           #if uf>0.5:
           #   uf=0.5
           #elif uf<-0.5:
           #   uf=-0.5
              
           ret=uf

           if self.index<self.FO_L:
              self.index=self.index+1
           else:
              self.index=0
          
           self.errsat[self.index]=u0-uf    

           return ret   
    
    #get index for derivative
    def get_index(self):
        return self.index

class Objtrack:
     def __init__(self):

        # First get an instance of the API endpoint (the connect via web case will be similar)
        self.api = local_connect()    #from droneapi.lib-->__init__.py,commented by ljx 

        # Our vehicle (we assume the user is trying to control the first vehicle attached to the GCS)
        self.vehicle = self.api.get_vehicles()[0]

        self.frame=None
        self.webframe=None
        self.timelast=time.time()

        #lamda=0.928  
        self.vx_fopi=fo_pi(1.3079,3.4269,4999)
        self.vy_fopi=fo_pi(1.3079,3.4269,4999)

        # horizontal velocity pid controller.  maximum effect is 10 degree lean
        xy_p = balloon_config.config.get_float('general','VEL_XY_P',1.0)
        xy_i = balloon_config.config.get_float('general','VEL_XY_I',0.0)
        xy_d = balloon_config.config.get_float('general','VEL_XY_D',0.0)
        xy_imax = balloon_config.config.get_float('general','VEL_XY_IMAX',10.0)
        self.vel_xy_pid = pid.pid(xy_p, xy_i, xy_d, math.radians(xy_imax))

        # vertical velocity pid controller.  maximum effect is 10 degree lean
        z_p = balloon_config.config.get_float('general','VEL_Z_P',2.5)
        z_i = balloon_config.config.get_float('general','VEL_Z_I',0.0)
        z_d = balloon_config.config.get_float('general','VEL_Z_D',0.0)
        z_imax = balloon_config.config.get_float('general','VEL_IMAX',10.0)
        self.vel_z_pid = pid.pid(z_p, z_i, z_d, math.radians(z_imax))

        # velocity controller min and max speed
        self.vel_speed_min = balloon_config.config.get_float('general','VEL_SPEED_MIN',1.0)
        self.vel_speed_max = balloon_config.config.get_float('general','VEL_SPEED_MAX',4.0)
        self.vel_speed_last = 0.0   # last recorded speed
        self.vel_accel = balloon_config.config.get_float('general','VEL_ACCEL', 0.5)    # maximum acceleration in m/s/s
        self.vel_dist_ratio = balloon_config.config.get_float('general','VEL_DIST_RATIO', 0.5)
     def object_find_hough(self,frame):
        object_found = False
        object_x = 0
        object_y = 0
        object_radius = 0
        circles=None
        
        img = cv2.GaussianBlur(frame,(3,3),0)
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)  
        #print'hough starts:' 
        #ret, binary = cv2.threshold(gray,127,255,cv2.THRESH_BINARY)  
        circles = cv2.HoughCircles(gray,cv2.cv.CV_HOUGH_GRADIENT,1,100,
                            param1=150,param2=50,minRadius=0,maxRadius=0)
        #print 'hough ends:'
        if circles is not None:
           
           circles = np.uint16(np.around(circles))
            # find largest circle
           if len(circles) > 0:
               print'len of circles:', len(circles)
               max_radius=0
               for i in circles[0,:]:
                   if i[2] > max_radius:
                       kp_max = i
                       
               if kp_max is not None and kp_max[2]>20 and kp_max[2]<200:
                  # draw the outer circle
                  cv2.circle(frame,(int(kp_max[0]),int(kp_max[1])),int(kp_max[2]),(255,0,0),4)
                  
                  # draw the center of the circle
                  #cv2.circle(frame,(kp_max[0],kp_max[1]),2,(0,0,255),3)
               
                  object_found=True
                  object_x = kp_max[0]
                  object_y = kp_max[1]
                  object_radius = kp_max[2]
               
        # return results
        return object_found, object_x, object_y, object_radius

 
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
        filter_low =np.array([150,90,30]) #for red circle
        filter_high=np.array([230,230,255])
        

        # Convert BGR to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Threshold the HSV image
        mask = cv2.inRange(hsv, filter_low, filter_high)
        
        # Erode
        erode_kernel = np.ones((3,3),np.uint8);
        eroded_img = cv2.erode(mask,erode_kernel,iterations = 1)
        
        # dilate
        dilate_kernel = np.ones((10,10),np.uint8);
        dilate_img = cv2.dilate(eroded_img,dilate_kernel,iterations = 1)
    
        # blob detector
        blob_params = cv2.SimpleBlobDetector_Params()
        blob_params.minDistBetweenBlobs = 50
        blob_params.filterByInertia = False
        blob_params.filterByConvexity = False
        blob_params.filterByColor = True
        blob_params.blobColor = 255
        blob_params.filterByCircularity = False
        blob_params.filterByArea = False
        #blob_params.minArea = 20
        #blob_params.maxArea = 500
        blob_detector = cv2.SimpleBlobDetector(blob_params)
        keypts = blob_detector.detect(dilate_img)
    
        # draw centers of all keypoints in new image
        blob_img = cv2.drawKeypoints(frame, keypts, color=(0,255,0), flags=0)
        
        cv2.circle(frame,(320,240),20,(0,255,0),2)
        #cv2.putText(frame,'Center: 320,240',(320,240), font, 1,(255,255,255),2)
        #cv2.putText(frame,'Object coordinate x:' + str(kp_max[0]) + "; y=" + str(kp_max[1]),(int(kp_max[0]),int(kp_max[1])), font, 1,(255,255,255),2)

        #adding text to image
        font = cv2.FONT_HERSHEY_SIMPLEX
        #Python:#cv2.putText(img, text, org, fontFace, fontScale, color[, thickness[, lineType[, bottomLeftOrigin]]]) 
        #cv2.putText(frame,'Love you!',(10,30), font, 1,(255,255,255),2)
        cv2.putText(frame,'100,100',(100,100),font, 0.3,(255,0,0),1)
        # find largest blob
        if len(keypts) > 0:
            kp_max = keypts[0]
            for kp in keypts:
                if kp.size > kp_max.size:
                    kp_max = kp
    
            if kp_max.size>5:
               # draw circle around the largest blob
               cv2.circle(frame,(int(kp_max.pt[0]),int(kp_max.pt[1])),int(kp_max.size),(0,0,255),6)
               #cv2.circle(frame,(320,240),10,(255,0,0),6)    
			   
               #cv2.line(img, Point pt1, Point pt2, color[,thickness[,lineType[,shift]]]) --> img
               cv2.line(frame,(320,240),(int(kp_max.pt[0]),int(kp_max.pt[1])),(255,0,0),5)
               
			   #adding text to image
               font = cv2.FONT_HERSHEY_SIMPLEX
               #Python:#cv2.putText(img, text, org, fontFace, fontScale, color[, thickness[, lineType[, bottomLeftOrigin]]]) 
               #cv2.putText(frame,'object',(int(kp_max.pt[0]),int(kp_max.pt[1])), font, 1,(255,255,255),2)
               cv2.putText(frame,'object x=' + str(kp_max.pt[0])[0:5] + "; y=" + str(kp_max.pt[1])[0:5],(int(kp_max.pt[0])-100,int(kp_max.pt[1])+10), font, 0.4,(255,255,255),1)

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
                                                     0b0000111111000111, # type_mask (only speeds enabled)
                                                     0, 0, 0, # x, y, z positions (not used)
                                                     #velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
                                                     vel_setx,vel_sety,0,#right side direction
                                                     0, 0, 0, # x, y, z acceleration (not used)
                                                     0, 0)    # yaw, yaw_rate (not used)
          # send command to vehicle

          self.vehicle.send_mavlink(msg)
          self.vehicle.flush()
          time.sleep(0.1)
     
     def run(self):

        yawangle=0.0
        imgnum=0
        bal_rc1=self.vehicle.parameters['RC1_TRIM']
        bal_rc2=self.vehicle.parameters['RC2_TRIM']
        RC7MAX=self.vehicle.parameters['RC7_MAX']
        RC7MIN=self.vehicle.parameters['RC7_MIN']  
        RC6MIN=self.vehicle.parameters['RC6_MIN']
        RC6MAX=self.vehicle.parameters['RC6_MAX']
        RC6MID=(RC6MIN+RC6MAX)/2
       # print "RC6MIN=",RC6MIN,",RC6MAX=",RC6MAX,",RC6MID=",RC6MID
        last_mode=None
        gain=1.0

        deltasatx=0
        deltasaty=0  
        

        try:
            
            web=Webserver(balloon_config.config.parser,(lambda:self.webframe))
            #Shang uncommented here. 
			#web=Webserver(balloon_config.config.parser,(lambda:self.frame))

            fourcc = cv2.cv.CV_FOURCC(*'XVID')
            video_name='video_%s.avi' % (time.time())
            video_writer = cv2.VideoWriter(video_name,fourcc, 1.0, (640,480))
            
            print "Initialising camera."
            # initialise video capture
            camera = balloon_video.get_camera()
            
            log_gname='BoTest%s.txt' % (time.time())
            gf = open(log_gname, mode='w')
            gf.write("Guided velocity mode log DATA %s\n"%(time.localtime()))
            gf.write("time\t yaw(org,radians)\t yaw_angle\t objposx\t objposy\t error_total\t vx\t vy\t gain\t imgradius\t roll\t pitch\t RC6VAL\n") #coeffx\t coeffy\t
            gf.close
            
 
            cycle_num=0
            coeffx=1
            coeffy=1
            
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

 
              #self.balloon_found, xpos, ypos, size = balloon_finder.analyse_frame(frame)
              self.object_found,xpos,ypos,size=self.object_find(newframe)
              #self.object_found,xpos,ypos,size=self.object_find_hough(frame)
              #if self.object_found:              
			  
              #adding text to image
              font = cv2.FONT_HERSHEY_SIMPLEX
              #Python:#cv2.putText(img, text, org, fontFace, fontScale, color[, thickness[, lineType[,bottomLeftOrigin]]]) 
              cv2.putText(newframe,'Object Found? ' + str(self.object_found)[0],(10,60), font, 0.3,(255,255,255),1)
		
			  
              #print 'New video frame. Time: %s' % (time.time())
              #print xpos
              
              RC6VAL=float(self.vehicle.channel_readback['6'])
              RC7VAL=self.vehicle.channel_readback['7']  
              if RC7VAL<=0 or RC7MAX is None:
                 gain=1.0
              else:
                 gain=7.0*(RC7VAL-RC7MIN)/(RC7MAX-RC7MIN)
              
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
			  
              gain1=0.05*gain
              gain=gain1
              #print 'gain is:',gain,'RC7=',RC7VAL

              #RC6VAL=1
              #if RC6VAL==1:
              if RC6VAL<RC6MID:#vision-controlled
                           
                 #print 'vision-controlled.'
				 
                 yaw_origin=self.vehicle.attitude.yaw
                 roll_origin=self.vehicle.attitude.roll
                 pitch_origin=self.vehicle.attitude.pitch
                 
                 cv2.line(newframe,(320,240),(int(320+100*math.cos(yaw_origin)),int(240+100*math.sin(yaw_origin))),(255,0,0),3)
                 cv2.putText(newframe,str(yaw_origin)[0:5],(int(320+100*math.cos(yaw_origin)),int(240+100*math.sin(yaw_origin))),font,0.5,(255,0,0),1)
                 if self.vehicle.mode.name=="GUIDED":#visual-guided
                    last_mode="GUIDED"

                    #print "---------------New period-----------------"

                    #print 'vehicle.mode = Guided. Gain =',gain
					
                    if self.object_found: 
                       print('object found')

                       pixelsize=0.03
                       deltax=xpos-320
					   #cv2.line(frame,(320,240),320)
                       deltay=240-ypos
                       #print 'yaw-origin: deltaxpos  deltaypos ', yaw_origin,deltax,deltay
                       print 'roll, pitch=', roll_origin,pitch_origin
					   
                       #print'deltax y'
                       #print deltax,deltay 
                       angle_yawtmp=0.5*3.14-math.atan2(deltay,deltax)+yaw_origin#radians
                       angle_yaw=wrap_PI(angle_yawtmp)
                       #print'angle1'
                       angle_yawdeg=math.degrees(angle_yaw)
                       
                       angle_pitch=0
                       speedSet=0.15

                       #spdfactor=math.sqrt((deltax*daltax+deltay*deltay))*0.1

                       realrx=deltax*pixelsize

                       realry=deltay*pixelsize
                       #spdfactor=((rx**2+ry**2)**0.5*0.01
 
                       spddist=math.sqrt(realrx**2+realry**2)

                       #print "dist:",spddist
                       #if spddist<=0.5:
                       #   self.vehicle.mode=VehicleMode("LAND")
                       #   self.vehicle.flush()



                       spdfactor=spddist*0.2
                       '''  
                       # get time since last time velocity pid controller was run
                       dt = self.vel_xy_pid.get_dt(2.0)
                       
                       # get speed towards balloon based on balloon   distance
                       #speed = self.balloon_distance * self.vel_dist_ratio
                       speed=spdfactor
                       # apply min and max speed limit
                       speed = min(speed, self.vel_speed_max)
                       speed = max(speed, self.vel_speed_min)

                       # apply acceleration limit
                       speed_chg_max = self.vel_accel * dt
                       speed = min(speed, self.vel_speed_last + speed_chg_max)
                       speed = max(speed, self.vel_speed_last - speed_chg_max)

                       # record speed for next iteration
                       self.vel_speed_last = speed
                       '''
                       cos_pitch=math.cos(angle_pitch)

                       velocity_x=gain*speedSet*math.cos(angle_yaw)*cos_pitch*spdfactor
                       velocity_y=float(gain*speedSet*math.sin(angle_yaw)*cos_pitch*spdfactor)
                       #velocity_x=speedSet*math.cos(angle_yaw)*cos_pitch*speed
                       #velocity_y=speedSet*math.sin(angle_yaw)*cos_pitch*speed
                       #velocity_z=speedSet*math.sin(angle_pitch)
                       velocity_z=0
                       #print 'angle and velocity....'
                       #print angle_yawdeg,velocity_x,velocity_y
                       
                       #start calculate the pid controller's term
                       dtx=self.vx_fopi.get_dt(2.0)
                                              
                       #errorx=math.sqrt(deltax*deltax+deltay*deltay)*math.cos(angle_yaw)*0.265/size#radius of the red circle is 25.5cm
                       #errorx=math.sqrt(deltax*deltax+deltay*deltay)*math.cos(angle_yaw)*0.265/(1*size)#radius of the red circle is 25.5cm
                       #errorx=math.sqrt(deltax*deltax+deltay*deltay)*math.cos(angle_yaw)*0.012/(1*size)#radius of the blue circle is 1.2cm
                       #errorx=errorx-deltasatx*0.3 #anti-wind
                       #errorx=errorx*gain
                       
                       error_total=math.sqrt(deltax*deltax+deltay*deltay)*0.265/(1*size)#radius of the blue circle is 1.2cm

                       error_x=error_total*math.cos(angle_yaw)
                       error_y=error_total*math.sin(angle_yaw)
                   
                          
                       #print'error_total,gain,angletoN',error_total,gain,angle_yaw
                       upx=error_total*math.cos(angle_yaw)*gain*coeffx
                       upy=error_total*math.sin(angle_yaw)*gain*coeffy
                       

                       upx_fo=self.vx_fopi.get_fopi(error_x, dtx)*gain
                       upy_fo=upx_fo/math.tan(angle_yaw)

                       
                       #print 'upx,upx_fo,upy,upy_fo to object',upx,upx_fo,upy,upy_fo

                       #print 'err_total,error_x',error_total,error_x                       

                       upx=upx_fo
                       upy=upy_fo
                       #print 'upx,upy to drone',upx,upy
                        
                       if upx>1.0:
                          upx=1.0
                          upy=upx/math.tan(angle_yaw)
                       elif upx<-1.0:
                          upx=-1.0
                          upy=upx/math.tan(angle_yaw)
                       if upy>1.0:
                          upy=1.0
                          upx=upy*math.tan(angle_yaw)
                       elif upy<-1.0:
                          upy=-1.0
                          upx=upy*math.tan(angle_yaw)

                       #errorx=velocity_x
                       
                       #upx=upx*gain
                       ''' 
                       dty=self.vy_fopi.get_dt(2.0)
                       #errory=velocity_y
                       #errory=math.sqrt(deltax*deltax+deltay*deltay)*math.sin(angle_yaw)*0.265/size
                       errory=math.sqrt(deltax*deltax+deltay*deltay)*math.sin(angle_yaw)*0.265/size
                       #errory=math.sqrt(deltax*deltax+deltay*deltay)*math.sin(angle_yaw)*0.012/size
                       #errory=errory-deltasaty*0.3 
                       errory=errory*gain
                       
                       print'errory,gain',errory,gain

                       upy=self.vy_fopi.get_fopi(errory, dty)
                       #upy=upy*gain
                       '''   
                       '''                       
                       print'gain,num,sampling time',gain,cycle_num,dtx


                       print 'control speed of x from fo-pi to p upx,velx,upy,vely',upx,velocity_x,upy,velocity_y
                       '''
                       self.vel_control(upx,upy)
                       time.sleep(0.1)
                 
                       deltatime=time.time()-self.timelast
                       gf = open(log_gname, mode='a')
                       gf.write("%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n"%(deltatime,yaw_origin,angle_yaw,xpos,ypos,error_total,upx,upy,gain,size,roll_origin,pitch_origin,RC6VAL))
                       gf.close()

                    else:#not-found
                         print "Guided mode,object not found. Gain = %f; Time = %f" % (gain,time.time())
                         self.vel_control(0,0)
                         #time.sleep(0.5)

                         RC6VAL=float(self.vehicle.channel_readback['6'])
                         deltatime=time.time()-self.timelast
                         gf = open(log_gname, mode='a')
                         gf.write("%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n"%(deltatime,0,0,-566,-566,0,0,0,0,size,roll_origin,pitch_origin,RC6VAL))
                         gf.close()

                 else:#non-guided
                        
                        print "vision control but non-guided. Yaw =", str(self.vehicle.attitude.yaw)

                        RC6VAL=float(self.vehicle.channel_readback['6'])
                        deltatime=time.time()-self.timelast
                        gf = open(log_gname, mode='a')
                        gf.write("%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n"%(deltatime,0,0,-566,-566,0,0,0,0,size,roll_origin,pitch_origin,RC6VAL))
                        gf.close()
                        
                        if last_mode=="GUIDED":
                            self.vel_control(0,0)
                            last_mode=self.vehicle.mode.name
                            time.sleep(0.5)

                                               
 
              else:#non-vision-controlled
                   
                   print "non-vision-controlled,landing......"

                   deltatime=time.time()-self.timelast
                   gf = open(log_gname, mode='a')
                   gf.write("%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n"%(deltatime,yaw_origin,angle_yaw,xpos,ypos,error_total,upx,upy,gain,size,RC6VAL,RC6VAL,RC6VAL))
                   gf.close()

                   while(self.vehicle.mode.name=="GUIDED"):

                             self.vehicle.mode.name="LAND"
                             self.vehicle.mode = VehicleMode("LAND")
                             self.vehicle.flush()
                             time.sleep(1)
                   while(self.vehicle.armed is True):
                             self.vehicle.armed=False
                             self.vehicle.flush()
                             time.sleep(1)
                      
              video_writer.write(newframe)
              self.webframe=newframe
      
              #time.sleep(1)  
        # handle interrupts
        except KeyboardInterrupt:
            print 'interrupted, exiting'

        # exit and close web server
        print "exiting..."
        web.close()
        #Shang uncommented here.

        video_writer.release()  


     def arm_and_takeoff():
        """Dangerous: Arm and takeoff vehicle - use only in simulation"""
        # NEVER DO THIS WITH A REAL VEHICLE - it is turning off all flight safety checks
        # but fine for experimenting in the simulator.
        print "Arming and taking off"
        self.vehicle.mode    = VehicleMode("STABILIZE")
        self.vehicle.parameters["ARMING_CHECK"] = 0
        self.vehicle.armed   = True
        self.vehicle.flush()

        while not self.vehicle.armed and not api.exit:
            print "Waiting for arming..."
            time.sleep(1)

            print "Taking off!"
            self.vehicle.commands.takeoff(20) # Take off to 20m height

            # Pretend we have a RC transmitter connected
            rc_channels = self.vehicle.channel_override
            rc_channels[3] = 1500 # throttle
            self.vehicle.channel_override = rc_channels

            self.vehicle.flush()
            time.sleep(10)

strat = Objtrack()
strat.run()

