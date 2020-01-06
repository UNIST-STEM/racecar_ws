import cv2
import numpy as np
import argparse

ap = argparse.ArgumentParser()
ap.add_argument("-s",
                "--source",
                type=int,
                help="optional argument for choosing a specific camera source (-1,0,1,2,3)")
args = vars(ap.parse_args())

###################### GLOBAL SYSTEM STATE ########################

# class for a generic data holder
class Data:
    def __init__(self): pass    # empty constructor
    
# object (of class Data) to hold the global state of the system
D = Data()

# do you want small windows? (about 320 x 240)
D.half_size = False

# all of the threshold keys
D.THRESH_KEYS = ['low_hue', 'high_hue', 'low_sat', 'high_sat', 'low_val', 'high_val']

current_mouse_pos = [0,0]
br = (0, 0, 0, 0)

#################### INITIALIZATION FUNCTIONS ######################

def createNewThresholds( list_of_values ):
    """ returns a dictionary d with (at least 12 keys)
        d['low_red']
        d['high_red']
        ... similar for green, blue, hue, sat, and val

        the input, list_of_values, should be at least
        12 numeric values indicating the low and the high
        for each channel, in that order
    """
    if len( list_of_values ) < 6:
        print ("Your input to createNewThresholds has too few values:")
        print (list_of_values)
    d = {}
    i = 0
    for key in D.THRESH_KEYS:  # loop through the keys
        d[key] = list_of_values[i]
        i += 1
    return d

def init_globals():
    """ sets up the data we need in the global dictionary D
    """
    """ sets up the data we need in the global dictionary D
    """
    # get D so that we can change values in it
    global D

    # Set up the windows containing the image from the camera,
    # the altered image, and the threshold sliders.
    cv2.namedWindow('My Webcam')
    cv2.moveWindow('My Webcam', 21, 21)

    cv2.namedWindow('Threshold')
    THR_WIND_OFFSET = 640
    if D.half_size: THR_WIND_OFFSET = int(THR_WIND_OFFSET/2)
    cv2.moveWindow('Threshold', THR_WIND_OFFSET, 21)

    #cv2.namedWindow('Sliders')
    cv2.namedWindow('Sliders', cv2.WINDOW_NORMAL)
    #SLD_WIND_OFFSET = 1280
    SLD_WIND_OFFSET = 660
    if D.half_size: SLD_WIND_OFFSET = int(SLD_WIND_OFFSET/2) 
    cv2.moveWindow('Sliders', SLD_WIND_OFFSET, 21)
    cv2.resizeWindow('Sliders', 600, 350)
    

    # put threshold values into D - hooray for Python lists!
    D.thresholds =  createNewThresholds( [0,255]*6 )  

    # Create the sliders within the 'Sliders' window
    cv2.createTrackbar('low_hue', 'Sliders', D.thresholds['low_hue'], 255, 
                          lambda x: change_slider('low_hue', x) )
    cv2.createTrackbar('high_hue', 'Sliders', D.thresholds['high_hue'], 255, 
                          lambda x: change_slider('high_hue', x) )
    cv2.createTrackbar('low_sat', 'Sliders', D.thresholds['low_sat'], 255, 
                          lambda x: change_slider('low_sat', x) )
    cv2.createTrackbar('high_sat', 'Sliders', D.thresholds['high_sat'], 255, 
                          lambda x: change_slider('high_sat', x) )
    cv2.createTrackbar('low_val', 'Sliders', D.thresholds['low_val'], 255, 
                          lambda x: change_slider('low_val', x) )
    cv2.createTrackbar('high_val', 'Sliders', D.thresholds['high_val'], 255, 
                          lambda x: change_slider('high_val', x) )

    # Set the method to handle mouse button presses
    cv2.setMouseCallback('My Webcam', onMouse)

    # We have not created our "scratchwork" images yet
    D.created_images = False

    # Variable for key presses
    D.last_key_pressed = 255

def blank_img(height, width, channel):
    """ Creates new empty numpy based image filled with zeros and holds 
        the inputted number of color channels
    """
    image = np.zeros((height, width, channel), np.uint8) 

    return image

def init_images():
    """ Creates all the images we'll need. Is separate from init_globals 
        since we need to know what size the images are before we can make
        them
    """
    # get D so that we can change values in it
    global D

    # Find the size of the image 
    # We set D.image right before calling this function
    try:
        D.height, D.width = D.image.shape[:2]
    except:
        pass

    # Create images for each color channel
    D.red = blank_img(D.height, D.width, 1)             
    D.blue = blank_img(D.height, D.width, 1)
    D.green = blank_img(D.height, D.width, 1)
    D.hue = blank_img(D.height, D.width, 1)
    D.sat = blank_img(D.height, D.width, 1)
    D.val = blank_img(D.height, D.width, 1)

    # Create images to save the thresholded images to
    D.red_threshed = blank_img(D.height, D.width, 1)
    D.green_threshed = blank_img(D.height, D.width, 1)
    D.blue_threshed = blank_img(D.height, D.width, 1)
    D.hue_threshed = blank_img(D.height, D.width, 1)
    D.sat_threshed = blank_img(D.height, D.width, 1)
    D.val_threshed = blank_img(D.height, D.width, 1)

    # The final thresholded result
    D.threshed_image = blank_img(D.height, D.width, 1)

    # Create an hsv image and a copy for contour-finding
    D.hsv = blank_img(D.height, D.width, 3)
    D.copy = blank_img(D.height, D.width, 1)

    # bunch of keypress values
    # So we know what to show, depending on which key is pressed
    # optional
    D.key_dictionary = {ord('w'): D.threshed_image,
                        ord('u'): D.red,
                        ord('i'): D.green,
                        ord('o'): D.blue,
                        ord('j'): D.red_threshed,
                        ord('k'): D.green_threshed,
                        ord('l'): D.blue_threshed,
                        ord('a'): D.hue,
                        ord('s'): D.sat,
                        ord('d'): D.val,
                        ord('z'): D.hue_threshed,
                        ord('x'): D.sat_threshed,
                        ord('c'): D.val_threshed,
                        }

    # set the default image for the second window
    D.current_threshold = D.threshed_image
  
################## END INITIALIZATION FUNCTIONS ####################

def threshold_image():
    """ runs the image processing in order to create a 
        black and white thresholded image out of D.image
        into D.threshed_image.
    """
    # get D so that we can change values in it
    global D, br

    # Use OpenCV to split the image up into channels, saving them in gray images
    D.blue, D.green, D.red = cv2.split(D.image)

    # This line creates a hue-saturation-value image
    D.hsv = cv2.cvtColor(D.image, cv2.COLOR_RGB2HSV)
    D.hue, D.sat, D.val = cv2.split(D.hsv)

    # Here is how OpenCV thresholds the images based on the slider values:
    cv2.inRange(D.hue, D.thresholds["low_hue"], D.thresholds["high_hue"], D.hue_threshed)
    cv2.inRange(D.sat, D.thresholds["low_sat"], D.thresholds["high_sat"], D.sat_threshed)
    cv2.inRange(D.val, D.thresholds["low_val"], D.thresholds["high_val"], D.val_threshed)

    #print "Threshes:"
    #print "Red:", D.thresholds["low_red"], D.thresholds["high_red"]

    # Multiply all the thresholded images into one "output" image, D.threshed_image
    cv2.multiply(D.sat_threshed, D.hue_threshed, D.threshed_image)
    cv2.multiply(D.threshed_image, D.val_threshed, D.threshed_image) 

    # Erode and Dilate shave off and add edge pixels respectively
    #cv2.Erode(D.threshed_image, D.threshed_image, iterations = 1)
    #cv2.Dilate(D.threshed_image, D.threshed_image, iterations = 1)

#
# This is the primary routine for doing pixel-processing work...
#

def find_biggest_region():
    """ finds all the contours in threshed image, finds the largest of those,
        and then marks in in the main image
    """
    # get D so that we can change values in it
    global D, br

    # call threshold_image in order to create D.threshed_image
    # use the passed-in thresholds
    threshold_image()

    # Create a copy image of thresholds then find contours on that image
    D.copy = D.threshed_image.copy() # copy threshed image

    # this is OpenCV's call to find all of the contours:
    #_,contours, hierarchy = cv2.findContours(D.copy, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours, hierarchy = cv2.findContours(D.copy, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours = sorted(contours, key = cv2.contourArea, reverse = True) #[:2]

    if len(contours) != 0:
            # draw in red the contours that were found
            #cv2.drawContours(D.image, contours, -1, (0, 0, 255), 3)

            # find the biggest area
            c = max(contours, key = cv2.contourArea)

            x,y,w,h = cv2.boundingRect(c)
            br = cv2.boundingRect(c)

            # draw the contour around red(in green)
            cv2.rectangle(D.image, (x,y), (x+w,y+h), (0,255,0), 2)

    # once everything is drawn, return the bounding rectangle
    return br

################# END IMAGE PROCESSING FUNCTIONS ###################

def onMouse(event, x, y, flags, param):
    """ the method called when the mouse is clicked """
    global D

    current_mouse_pos[0] = x
    current_mouse_pos[1] = y

    # clicked the left button
    if event == cv2.EVENT_LBUTTONDOWN: 
        print ("x, y are", x, y, "    ",)
        (b,g,r) = D.image[y,x]
        print ("r,g,b is", int(r), int(g), int(b), "    ",)
        D.hsv = cv2.cvtColor(D.image, cv2.COLOR_RGB2HSV)
        (h,s,v) = D.hsv[y,x]
        print ("h,s,v is", int(h), int(s), int(v))
        D.down_coord = (x,y)

def check_key_press(key_press):
    """ this handler is called when a real key press has been
        detected, and updates everything appropriately
    """
    # get D so that we can change values in it
    global D
    D.last_key_pressed = key_press

    # if it was ESC, make it 'q'
    if key_press == 27:
        key_press = ord('q')

    # help menu
    if key_press == ord('h'):
        print (" Keyboard Command Menu")
        print (" ==============================")
        print (" q    : quit")
        print (" ESC  : quit")
        print (" h    : help menu")
        print (" w    : show total threshold image in threshold window")
        print (" r    : show red image in threshold window")
        print (" t    : show green image in threshold window")
        print (" y    : show blue image in threshold window")
        print (" f    : show thresholded red image in threshold window")
        print (" g    : show thresholded blue image in threshold window")
        print (" h    : show thresholded green image in threshold window")
        print (" a    : show hue image in threshold window")
        print (" s    : show saturation image in threshold window")
        print (" d    : show value image in threshold window")
        print (" z    : show thresholded hue image in threshold window")
        print (" x    : show thresholded saturation image in threshold window")
        print (" c    : show thresholded value image in threshold window")
        print (" m    : saves threshold values to file (w/ name)")
        print (" l    : loads threshold values from file")


    elif key_press == ord('m'):
        filename = "thresh.txt"
        f = open( filename, "w" ) # open the file "thresh.txt" for writing
        print >> f, D.thresholds # print them to the file object f
        f.close() # it's good to close the file afterwards
        print ("(M) Wrote thresholds to thresh.txt. Use 'L' to load them.")
        print ("++ Remember - it's a good idea to copy those to a backup name ++")

    elif key_press == ord('l'):
        filename = "thresh.txt"
        # should check if file exists!
        try:
            f = open( filename, "r" ) # open the file "thresh.txt" for reading
            data = f.read() # read everything from f into data
            D.thresholds = eval( data ) # eval is Python's evaluation function
            # eval evaluates strings as if they were at the Python shell
            f.close() # its good to close the file afterwards
            print ("(L) Loaded thresholds from thresh.txt. Use 'M' to save them.")
            # Update threshold values on actual sliders
            for key in D.THRESH_KEYS:
                cv2.setTrackbarPos(key, 'Sliders', D.thresholds[key])
        except:
            print ("Could not find the file", filename, "Not loaded...")
        
    # threshold keypresses:
    elif key_press in D.key_dictionary.keys():
        D.current_threshold = D.key_dictionary[key_press]


# Function for changing the slider values
def change_slider(name, new_threshold):
    """ a small function to change a slider value """
    # get D so that we can change values in it
    global D
    D.thresholds[name] = new_threshold

    # Update the displays:
    # Main image:
    cv2.imshow('My Webcam', D.image)

    # Currently selected threshold image:
    cv2.imshow('Threshold', D.current_threshold )


def handle_image():
    """ this function organizes all of the processing
        done for each image from a camera or Kinect
    """
    # get D so that we can change values in it
    global D

    if D.image is None : # did we get an image at all?
        print ("No image")
        return

    # handle different image sizes and image setup
    if D.half_size == False: # keep the image size
        D.image = D.image
    else: # halve the image size
        if D.created_images == False:
            w, h = D.image.width, D.image.height
            D.half_sz = (w/2,h/2)
            D.image.size = D.half_size
        cv2.resize(D.image,D.image)

    if D.created_images == False:   # have we set up the other images?
        init_images()               # Initialize the others needed
        D.created_images = True     # We only need to run this one time

    # Recalculate blob in main image using the global thresholds
    br = find_biggest_region()

    # Handle any incoming keypresses
    # To get input from keyboard, we use cv.waitKey
    # Only the lowest eight bits matter (so we get rid of the rest):
    key_press_raw = cv2.waitKey(1) # gets a raw key press
    key_press = key_press_raw & 255 # sets all but the low 8 bits to 0
    
    # Handle key presses only if it's a real key (255 = "no key pressed")
    if (key_press != 255):
        check_key_press(key_press)


    # Update the displays:
    # Main image:
    cv2.imshow('My Webcam', D.image)

    # Currently selected threshold image:
    cv2.imshow('Threshold', D.current_threshold )



def handle_camera_data(data):
    """ this function grabs images from a webcamera
    """
    # get D so that we can change values in it
    global D
    
    # Reads in source for cam feed if specific source is passed in
    try:
        cam = cv2.VideoCapture(0) 

    except:
        raise Exception("Pass in a video source")
        
    while True:

        ret_val, D.image = cam.read()
        
        # Option of resizing displayed window
        #D.image = cv2.resize(D.image, (0,0), fx = 1.5, fy = 1.5)
        
        handle_image()
        init_images()

        # regular webcam window must be selected
        if (cv2.waitKey(1) == 27): 
            break  # esc to quit
            
    cv2.destroyAllWindows()

        
def main():
    global D

    # Initialize all the global variables we will need
    init_globals()

    # Initialize camera feed
    handle_camera_data(0)


if __name__ == "__main__":
    main()
