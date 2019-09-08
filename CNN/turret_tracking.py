#!/usr/bin/python

# Usage: turret_tracking.py [options]
#
# Options:
#   -h, --help            show this help message and exit
#   -d, --disarm          track movement but do not fire
#   -r, --reset           reset the turret position and exit
#   --nd, --no-display    do not display captured images
#   -c NUM, --camera=NUM  specify the camera # to use. Default: 0
#   -s WIDTHxHEIGHT, --size=WIDTHxHEIGHT
#                         image dimensions (recommended: 320x240 or 640x480).
#                         Default: 320x240
#   -v, --verbose         detailed output, including timing information

import os
import sys
import time
import usb.core
import cv2
import subprocess
import shutil
import math
import threading
from optparse import OptionParser

# globals
FNULL = open(os.devnull, 'w')

class AttributeDict(dict):
    __getattr__ = dict.__getitem__
    __setattr__ = dict.__setitem__

class Launcher(): # a parent class for our low level turret.  
#Contains general movement commands which may be overwritten in case of hardware specific tweaks.
            
    # roughly centers the turret at the origin
    def center(self, x_origin=0.5, y_origin=0.5):
        print 'Centering camera ...'
        self.moveToPosition(x_origin,y_origin)

    def moveToPosition(self, right_percentage, down_percentage): 
        self.turretLeft()
        time.sleep( self.x_range)
        self.turretRight()
        time.sleep( right_percentage * self.x_range)
        self.turretStop()

        self.turretUp()
        time.sleep( self.y_range)
        self.turretDown()
        time.sleep( down_percentage * self.y_range) 
        self.turretStop()

    def moveRelative(self, right_percentage, down_percentage):
        if (right_percentage>0):
            self.turretRight()
        elif(right_percentage<0):
            self.turretLeft()
        time.sleep( abs(right_percentage) * self.x_range)
        self.turretStop()
        if (down_percentage>0):
            self.turretDown()
        elif(down_percentage<0):
            self.turretUp()
        time.sleep( abs(down_percentage) * self.y_range)
        self.turretStop()

# Launch commands for water cannon
class WaterCannon(Launcher):
		#TODO: USE THIS AS TEMPLATE FOR ARDUINO CONNECTION
    def __init__(self):
        self.dev = usb.core.find(idVendor=0x2123, idProduct=0x1010)

        # HID detach for Linux systems...tested with 0x2123 product

        if self.dev is None:
            raise ValueError('Water cannon not found.')

        #experimentally estimated speed scaling factors 
        self.y_speed = 0.48
        self.x_speed = 1.2    
        #approximate number of seconds of movement to reach end of range  
        self.x_range = 6.5  # this turret has a 270 degree range of motion and if this value is set
                            # correcly should center to be facing directly away from the usb cable on the back
        self.y_range = 0.75

        #define directional constants        
        self.DOWN = 0x01
        self.UP = 0x02
        self.LEFT = 0x04
        self.RIGHT = 0x08



    def turretUp(self):
        self.dev.ctrl_transfer(0x21, 0x09, 0, 0, [0x02, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])

    def turretDown(self):
        self.dev.ctrl_transfer(0x21, 0x09, 0, 0, [0x02, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])

    def turretLeft(self):
        self.dev.ctrl_transfer(0x21, 0x09, 0, 0, [0x02, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])

    def turretRight(self):
        self.dev.ctrl_transfer(0x21, 0x09, 0, 0, [0x02, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])

    def turretDirection(self,direction):
        self.dev.ctrl_transfer(0x21, 0x09, 0, 0, [0x02, direction, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])

    def turretStop(self):
        self.dev.ctrl_transfer(0x21, 0x09, 0, 0, [0x02, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])

    def turretFire(self):
        self.dev.ctrl_transfer(0x21, 0x09, 0, 0, [0x02, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])

    def ledOn(self):
        self.dev.ctrl_transfer(0x21, 0x09, 0, 0, [0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])

    def ledOff(self):
        self.dev.ctrl_transfer(0x21, 0x09, 0, 0, [0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])

 

class Turret():
    def __init__(self, opts):
        self.opts = opts

        self.launcher = WaterCannon();

        self.origin_x, self.origin_y = map(float, opts.origin.split(','))

        self.killcam_count = 0
        self.trackingTimer = time.time()
        self.locked_on = 0 

        # initial setup
        self.center()
        self.launcher.ledOff()
        if (opts.mode == "sweep"):
            self.approx_x_position = self.origin_x
            self.approx_y_position = self.origin_y
            self.sweep_x_direction = 1
            self.sweep_y_direction = 1
            self.sweep_x_step = 0.05
            self.sweep_y_step = 0.2

    # turn off turret properly
    def dispose(self):
        self.launcher.turretStop()
        turret.launcher.ledOff()

    # roughly centers the turret to the middle of range or origin point if specified
    def center(self):
        self.launcher.center(self.origin_x, self.origin_y)

    # adjusts the turret's position (units are fairly arbitary but work ok)
    def adjust(self, right_dist, down_dist):
        right_seconds = right_dist * self.launcher.x_speed
        down_seconds = down_dist * self.launcher.y_speed

        directionRight=0
        directionDown=0
        if right_seconds > 0:
            directionRight = self.launcher.RIGHT
        elif right_seconds < 0:
            directionRight = self.launcher.LEFT

        if down_seconds > 0:
            directionDown = self.launcher.DOWN
        elif down_seconds < 0:
            directionDown = self.launcher.UP

        #move diagonally first
        self.launcher.turretDirection(directionDown | directionRight) 

        #move remaining distance in one direction
        if (abs(right_seconds)>abs(down_seconds)):
            time.sleep(abs(down_seconds))
            self.launcher.turretDirection(directionRight)
            time.sleep(abs(right_seconds-down_seconds))            
        else:
            time.sleep(abs(right_seconds))
            self.launcher.turretDirection(directionDown)
            time.sleep(abs(down_seconds-right_seconds))          
        
        self.launcher.turretStop()

        # OpenCV takes pictures VERY quickly, so if we use it, we must
        # add an artificial delay to reduce camera wobble and improve clarity
        time.sleep(.2)

    #stores images of the targets within the killcam folder
    def killcam(self, camera):
        # create killcam dir if none exists, then find first unused filename
        if not os.path.exists("killcam"):
            os.makedirs("killcam")
        filename_locked_on = os.path.join("killcam", "lockedon" + str(self.killcam_count) + ".jpg")
        while os.path.exists(filename_locked_on):
            self.killcam_count += 1
            filename_locked_on = os.path.join("killcam", "lockedon" + str(self.killcam_count) + ".jpg")

        # save the image with the target being locked on

        cv2.imwrite(filename_locked_on, camera.frame_mod)

        # wait a little bit to attempt to catch the target's reaction.
        time.sleep(1)  
        camera.new_frame_available = False #force camera to obtain image after this point

        # take another picture of the target while it is being fired upon
        filename_firing = os.path.join("killcam", "firing" + str(self.killcam_count) + ".jpg")
        camera.face_detect(filename=filename_firing) 
        if not opts.no_display:
            camera.display()

        self.killcam_count += 1

    # compensate vertically for distance to target
    def projectile_compensation(self, target_y_size):
        if target_y_size > 0:
            # objects further away will need a greater adjustment to hit target
            adjust_amount = 0.1 * math.log(target_y_size)
        else:
            # log 0 will throw an error, so handle this case even though unlikely to occur
            adjust_amount = 0

        # tilt the turret up to try to increase range
        self.adjust(0, adjust_amount)
        if opts.verbose:
            print "size of target: %.6f" % target_y_size
            print "compensation amount: %.6f" % adjust_amount

    # turn on LED if face detected in range, and fire missiles if armed
    def ready_aim_fire(self, x_adj, y_adj, target_y_size, squirrel_detected, camera=None):
        fired = False
        if squirrel_detected and abs(x_adj) < .05 and abs(y_adj) < .05:
            turret.launcher.ledOn()  # LED will turn on when target is locked
            if self.opts.armed:
                # aim a little higher if our target is in the distance
                self.projectile_compensation(target_y_size)

                turret.launcher.turretFire()
                fired = True

                if camera:
                    self.killcam(camera)  # save a picture of the target

                time.sleep(3)  # disable turret for approximate time required to fire

                print 'Shots fired!'

            else:
                print 'Turret trained but not firing because of the --disarm directive.'
        else:
            turret.launcher.ledOff()
        return fired

    #keeps track of length of time since a target was found or lost
    def updateTrackingDuration(self, is_locked_on):
        
        if is_locked_on:
            if self.locked_on:
                trackingDuration = time.time() - self.trackingTimer
            else:
                self.locked_on = True
                self.trackingTimer = time.time()
                trackingDuration = 0
        else: #not locked on
            if self.locked_on:
                self.locked_on = False
                self.trackingTimer = time.time()
                trackingDuration = 0
            else:
                trackingDuration = -(time.time() - self.trackingTimer)
        return trackingDuration #negative values indicate time since target seen

    #increments the sweeping behaviour of a turret on patrol
    def sweep(self):
        self.approx_x_position += self.sweep_x_direction * self.sweep_x_step
        if(self.approx_x_position<=1 and self.approx_x_position>=0): 
            #move in x direction first
            turret.launcher.moveRelative(self.sweep_x_step * self.sweep_x_direction, 0)
        else:
            #reached end of x range.  move in y direction and switch x sweep direction
            self.sweep_x_direction = -1 * self.sweep_x_direction
            self.approx_x_position += self.sweep_x_direction * self.sweep_x_step 
            self.approx_y_position += self.sweep_y_direction * self.sweep_y_step
            if(self.approx_y_position<=1 and self.approx_y_position>=0): 
                #take a step in current y direction
                self.launcher.moveRelative(0, 0.2 * self.sweep_y_direction)
            else:
                #swap y direction and take a step in that direction instead
                self.sweep_y_direction = -1 * self.sweep_y_direction
                self.approx_y_position += self.sweep_y_direction * 2 * self.sweep_y_step # reverse previous y step and take a new step 
                self.launcher.moveRelative(0, self.sweep_y_step * self.sweep_y_direction)
        time.sleep(.2) #allow camera to stabilize


class Camera():
    def __init__(self, opts):
        self.opts = opts
        self.current_image_viewer = None  # image viewer not yet launched

        self.webcam = cv2.VideoCapture(int(self.opts.camera))  # open a channel to our camera
        if(not self.webcam.isOpened()):  # return error if unable to connect to hardware
            raise ValueError('Error connecting to specified camera')

        #if supported by camera set image width and height to desired values
        img_w, img_h = map(int, self.opts.image_dimensions.split('x'))
        self.resolution_set = self.webcam.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH,img_w)
        self.resolution_set =  self.resolution_set  and self.webcam.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT,img_h)         

        # create a separate thread to grab frames from camera.  This prevents a frame buffer from filling up with old images
        self.camThread = threading.Thread(target=self.grab_frames)
        self.camThread.daemon = True
        self.currentFrameLock = threading.Lock()
        self.new_frame_available = False
        self.camThread.start()

    # turn off camera properly
    def dispose(self):
        if sys.platform == 'linux2' or sys.platform == 'darwin':
            if self.current_image_viewer:
                subprocess.call(['killall', self.current_image_viewer], stdout=FNULL, stderr=FNULL)
        else:
            self.webcam.release()


    # runs to grab latest frames from camera
    def grab_frames(self):
            while(1): # loop until process is shut down
                if not self.webcam.grab():
                    raise ValueError('frame grab failed')
                time.sleep(.015)
                retval, most_recent_frame = self.webcam.retrieve(channel=0)
                if not retval:
                    raise ValueError('frame capture failed')
                self.currentFrameLock.acquire()
                self.current_frame = most_recent_frame
                self.new_frame_available = True
                self.currentFrameLock.release()
                time.sleep(.015)


    # run recognition on our previously captured image and returns
    # (x,y)-distance between target and center (as a fraction of image dimensions)
    def squirrel_detect(self, filename=None):
        def draw_reticule(img, x, y, width, height, color, style="corners"):
            w, h = width, height
            if style == "corners":
                cv2.line(img, (x, y), (x+w/3, y), color, 2)
                cv2.line(img, (x+2*w/3, y), (x+w, y), color, 2)
                cv2.line(img, (x+w, y), (x+w, y+h/3), color, 2)
                cv2.line(img, (x+w, y+2*h/3), (x+w, y+h), color, 2)
                cv2.line(img, (x, y), (x, y+h/3), color, 2)
                cv2.line(img, (x, y+2*h/3), (x, y+h), color, 2)
                cv2.line(img, (x, y+h), (x+w/3, y+h), color, 2)
                cv2.line(img, (x+2*w/3, y+h), (x+w, y+h), color, 2)
            else:
                cv2.rectangle(img, (x, y), (x+w, y+h), color)

        # load image, then resize it to specified size
        while(not self.new_frame_available):
            time.sleep(.001)
        self.currentFrameLock.acquire()
        img = self.current_frame.copy()
        self.new_frame_available = False
        self.currentFrameLock.release()

        img_w, img_h = map(int, self.opts.image_dimensions.split('x'))
        if(not self.resolution_set):
            img = cv2.resize(img, (img_w, img_h))

#TODO: REPLACE WITH SQUIRREL DETECTION
        # detect faces (might want to make the minNeighbors threshold adjustable)
        #squirrels = self.face_filter.detectMultiScale(img, minNeighbors=4)

        if self.opts.verbose:
            print 'Squirrel detected: ' + str(squirrels)

        x_adj, y_adj = (0, 0)  # (x,y)-distance from center, as a fraction of image dimensions
        squirrel_y_size = 0  # height of the bounding box, used to gauge distance to target
        if len(squirrels) > 0:
            face_detected = True

            draw_reticule(img, x, y, w, h, (0, 0, 170), "corners")
            x_adj = ((x + w/2) - img_w/2) / float(img_w)
            y_adj = ((y + h/2) - img_h/2) / float(img_h)
            face_y_size = h / float(img_h)
        else:
            squirrel_detected = False


        #store modified image as class variable so that display() can access it
        self.frame_mod = img
        if filename:    #save to file if desired
            cv2.imwrite(filename, img)

        return squirrel_detected, x_adj, y_adj, squirrel_y_size

    # display the OpenCV-processed images
    def display(self):
            #not tested on Mac, but the openCV libraries should be fairly cross-platform
            cv2.imshow("cameraFeed", self.frame_mod)

            # delay of 2 ms for refreshing screen (time.sleep() doesn't work)
            cv2.waitKey(2)

if __name__ == '__main__':
    if (sys.platform == 'linux2' or sys.platform == 'darwin') and not os.geteuid() == 0:
        sys.exit("Script must be run as root.")

    # command-line options
    parser = OptionParser()
    parser.add_option("-d", "--disarm", action="store_false", dest="armed", default=True,
                      help="track squirrels but do not fire water cannon")
    parser.add_option("-r", "--reset", action="store_true", dest="reset_only", default=False,
                      help="reset the turret position and exit")
    parser.add_option("--nd", "--no-display", action="store_true", dest="no_display", default=False,
                      help="do not display captured images")
    parser.add_option("-c", "--camera", dest="camera", default='0',
                      help="specify the camera # to use. Default: 0", metavar="NUM")
    parser.add_option("-s", "--size", dest="image_dimensions", default='320x240',
                      help="image dimensions (recommended: 320x240 or 640x480). Default: 320x240",
                      metavar="WIDTHxHEIGHT")
    parser.add_option("-v", "--verbose", action="store_true", dest="verbose", default=False,
                      help="detailed output, including timing information")    
    parser.add_option("-m", "--mode", dest="mode", default="follow",
                      help="choose behaviour of sentry. options (follow, sweep, guard) default:follow", metavar="NUM")      
    parser.add_option("-o", "--origin", dest="origin", default="0.5,0.5",
                      help="direction to point initially - an x and y decimal percentage. Default: 0.5,0.5", metavar="X,Y")    
    opts, args = parser.parse_args()
    print opts

    # additional options
    opts = AttributeDict(vars(opts))  # converting opts to an AttributeDict so we can add extra options

    turret = Turret(opts)
    camera = Camera(opts)
    turretCentered = True

    while (not camera.new_frame_available):
        time.sleep(.001)   #wait for first frame to be captured
    if not opts.reset_only:
        while True:
            try:
                start_time = time.time()
                squirrel_detected, x_adj, y_adj, face_y_size = camera.squirrel_detect()
                detection_time = time.time()

                if not opts.no_display:
                    camera.display()

                trackingDuration = turret.updateTrackingDuration(squirrel_detected)

                #if target is already centered in sights take the shot
                turret.ready_aim_fire(x_adj, y_adj, squirrel_y_size, squirrel_detected, camera) 
               
                if squirrel_detected:  
                    #face detected: move turret to track         
                    if opts.verbose:
                        print "adjusting turret: x=" + str(x_adj) + ", y=" + str(y_adj)
                    turret.adjust(x_adj, y_adj)
                    turretCentered=False
                elif (opts.mode=="guard") and (trackingDuration < -10) and (not turretCentered):
                    #If turret is in guard mode and has lost track of its target it should reset to the position it is guarding
                    turret.center()
                    turretCentered=True
                elif(opts.mode=="sweep") and (trackingDuration < -3):
                    turret.sweep()


                movement_time = time.time()
                camera.new_frame_available = False #force camera to obtain next image after movement has completed

                if opts.verbose:
                    print "total time: " + str(movement_time - start_time)
                    print "detection time: " + str(detection_time - start_time)
                    print "movement time: " + str(movement_time - detection_time)


            except KeyboardInterrupt:
                turret.dispose()
                camera.dispose()
                break

