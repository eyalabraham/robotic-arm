#!/usr/bin/python
###############################################################################
# 
# picknstack.py
#
#   Python OpenCV program to identify red/blue/green blocks on a white page,
#   determine their orientation angle and x,y position coordinate, and their color.
#   Feed this information to a robotics arm controller that will pick the blocks
#   and stack them in sorted piles.
#
#   Resources:
#       Thresholding: https://docs.opencv.org/3.2.0/d7/d4d/tutorial_py_thresholding.html
#       Contour detection: http://answers.opencv.org/question/179004/how-to-detect-object-and-pattern-and-retrieve-the-angle-of-orientation/
#       Detection in ROI: https://stackoverflow.com/questions/42004652/how-can-i-find-contours-inside-roi-using-opencv-and-python
#       Erode and Dilate: https://www.learnopencv.com/blob-detection-using-opencv-python-c/
#       Object color: https://www.pyimagesearch.com/2016/02/15/determining-object-color-with-opencv/
#
#   June 28, 2018
#
###############################################################################

import numpy as np
import cv2
import math
import time

import oarmik as ik
import oarmop

# These modules can be found here: https://github.com/eyalabraham/computer-vision
import opencvconst as cv
import colorlabeler

def main():
    """Identify blocks on a white page, their position, orientation, and color."""
    
    #
    # initializations
    #
    MODE_SHOW = 0                   # show identified block on webcam view
    MODE_PICK = 1                   # activate robotic arm to pick-n-place
    mode = MODE_SHOW
    roi_show = False
    
    mode_text = 'Mode: Show'
    fps_text = '? Fps'
    block_count_text = 'Blocks: ?'
    blocks = []

    ROI_XY   = (10,20,620,450)      # In the form of (x0,y0,x1,y1) TODO pick with mouse
    ORIGIN_PIX = (421,475)          # Robot arm grid coordinate origin TODO mark with mouse
    REFERENCE_MARKER = (421,135)
    ORIGIN_OFFSET = (100,0)         # Offset of arm's (0,0) from location of ORIGIN_PIX
    PIX_PER_MM = 2.24               # Robot arm grid coordinate resolution in pixels per mm
    GRIP_ROTATE_LIMIT = 60.0
    
    RED_BIN = (120,-160,10,-45)     # Drop locations for block colors 
    BLUE_BIN = (145,-120,10,-50)
    GREEN_BIN = (170,-160,10,-50)

    block_stack = {}                # Block stack count in bin

    GRIPPER_LEGO_WIDTH = 105        # Some constants
    GRIPPER_RELEASE = 115
    GRIPPER_OPEN = 160
    BRICK_DROP_PICK_HEIGHT = 5
    BRICK_HEIGHT = 12

    # Initialize the inverse kinematic model
    model = ik.OARMIK()
    
    # Kernel for dilation or erosion
    dilate_kernel = np.ones((5, 5), np.uint8)
    erode_kernel = np.ones((5, 5), np.uint8)
    
    # Initialize display window and font
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.namedWindow('blocks', cv2.WINDOW_NORMAL+cv2.WINDOW_AUTOSIZE)

    # Initialize start time and frame count
    frame_count = 0
    start = time.time()
    
    # Open the webcam device
    cap = cv2.VideoCapture(0)
    
    # Initialize color detector class
    cl = colorlabeler.ColorLabeler(color_dictionary = {"red": (100, 30, 50),"green": (25, 65, 45),"blue": (24, 43, 105),"orange": (160, 90, 50)})
    
    print ' s - stop, home arm, and show detected blocks\n p - activate robotic arm to pick-n-stack\n r - toggle display of ROI\n ESC - quit'
    
    #
    # frame capture and processing loop
    #
    while(True):

        cap_ok, img = cap.read()
        if not cap_ok:
            break

        #
        # Detect blocks in ROI
        #
        if mode == MODE_SHOW:
        
            # Process the image's gray scale to "clean up" noise
            blur = cv2.GaussianBlur(img, (5,5), 0)
            img_Lab = cv2.cvtColor(blur, cv2.COLOR_BGR2LAB)
            gray_image = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)
            ret,thresh = cv2.threshold(gray_image,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
            mask_erode = cv2.erode(thresh, erode_kernel, iterations = 1)
            mask = cv2.dilate(mask_erode, dilate_kernel, iterations = 3)

            # Detect contours
            im2, contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            
            block_count = 0
            blocks = []
            
            for cnt in contours:
                rect = cv2.minAreaRect(cnt)     # rect is a Box2D: center (x,y), (width, height), angle of rotation of the box
                box = np.asarray(cv2.boxPoints(rect),dtype=np.int0)
                contour_area = int(rect[1][0] * rect[1][1])
                
                b = box.transpose()

                # Select contours within the ROI
                if (min(b[0]) >= ROI_XY[0] and min(b[0]) <= ROI_XY[2]  and \
                    max(b[0]) >= ROI_XY[0] and max(b[0]) <= ROI_XY[2]) and \
                   (min(b[1]) >= ROI_XY[1] and min(b[1]) <= ROI_XY[3]  and \
                    max(b[1]) >= ROI_XY[1] and max(b[1]) <= ROI_XY[3]) and \
                   (contour_area >= 1500 and contour_area <= 2500):
                   
                    # Calculate contour parameters in arm coordinate space
                    # (x,y) coordinates
                    px, py = int(rect[0][0]), int(rect[0][1])
                    y, x = ((ORIGIN_PIX[0] - px) / PIX_PER_MM) + ORIGIN_OFFSET[1], ((ORIGIN_PIX[1] - py) / PIX_PER_MM) + ORIGIN_OFFSET[0]
                    # Gripper rotation
                    contour_rotation = get_rotation(box)
                    if contour_rotation > 0:
                        block_rotation = 90.0 - contour_rotation
                    else:
                        block_rotation = -90.0 - contour_rotation
                    gripper_roration = block_rotation - (57.29578049 * math.atan(y/x))    # Compensate for arm turret rotation
                    # Color
                    block_color = color = cl.label(img_Lab, cnt)
                    # Enumeration
                    block_count = block_count + 1

                    # Build a list of blocks to pick
                    block = [block_count,int(x),int(y),int(gripper_roration),block_color]
                    blocks.append(block)
                    
                    # Display contour parameters
                    cv2.drawContours(img,[box],0,(0,255,0),1)
                    cv2.putText(img, '({:.1f},{:.1f}) [mm]'.format(x,y), (px+40, py), font, 0.3, (255, 0, 0), 1, cv2.LINE_AA)
                    cv2.putText(img, 'B{:.0f}/G{:.0f} [deg]'.format(block_rotation,gripper_roration), (px+40, py+10), font, 0.3, (255, 0, 0), 1, cv2.LINE_AA)
                    cv2.putText(img, block_color, (px+40, py+20), font, 0.3, (255, 0, 0), 1, cv2.LINE_AA)

        #
        # Pick blocks mode
        #
        elif mode == MODE_PICK:
        
            with oarmop.OARMOP() as oArm:
    
                oArm.home()
                block_stack.clear()
            
                for block in blocks:
                
                    if block[4] not in ('red','green','blue'):
                        continue

                    # Move arm to position above block
                    result = move_arm(oArm, model, block[1], block[2], 50, block[3])
                    if result == False:
                        print 'err: Overrun.'
                        continue

                    # Lower arm to block and close gripper
                    oArm.set_gripper(GRIPPER_OPEN)
                    move_arm(oArm, model, block[1],block[2],BRICK_DROP_PICK_HEIGHT, block[3])
                    oArm.set_gripper(GRIPPER_LEGO_WIDTH)
                    
                    # Raise arm, then move to midpoint position above he 'blue' bin
                    move_arm(oArm, model, block[1], block[2], 50, block[3])
                    move_arm(oArm, model, BLUE_BIN[0], BLUE_BIN[1], 60, BLUE_BIN[3])

                    # Count blocks on the stack per color bin
                    if block[4] in block_stack:
                        block_stack[block[4]] = block_stack[block[4]] + 1
                    else:
                        block_stack[block[4]] = 0
                    
                    # Move arm to bin position and drop brick
                    block_drop_height = BRICK_DROP_PICK_HEIGHT + block_stack[block[4]] * BRICK_HEIGHT
                    
                    if block[4] == 'red':
                        move_arm(oArm, model, RED_BIN[0], RED_BIN[1], block_drop_height, RED_BIN[3])
                    elif block[4] == 'blue':
                        move_arm(oArm, model, BLUE_BIN[0], BLUE_BIN[1], block_drop_height, BLUE_BIN[3])
                    elif block[4] == 'green':
                        move_arm(oArm, model, GREEN_BIN[0], GREEN_BIN[1], block_drop_height, GREEN_BIN[3])
                    
                    oArm.set_gripper(GRIPPER_RELEASE)
                    
                    move_arm(oArm, model, BLUE_BIN[0], BLUE_BIN[1], 60, BLUE_BIN[3])
                    #time.sleep(3)
                    
                    oArm.home('arm')
                    oArm.home('boom')
                    oArm.home('rotate')

                    # An opportunity to exit 'pick' mode
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('s'):
                        break

                mode = MODE_SHOW
                mode_text = 'Mode: Show'

        #
        # Calculate and display FPS.
        #
        frame_count = frame_count + 1
        end = time.time()
        measure_interval = end - start
        if measure_interval > 10:
            fps = frame_count / measure_interval
            fps_text = '{:.2f} Fps'.format(fps)
            frame_count = 0
            start = time.time()

        #
        # Display formatting
        #
        block_count_text = 'Blocks: {}'.format(block_count)
        
        if roi_show:
            cv2.rectangle(img, (ROI_XY[0],ROI_XY[1]), (ROI_XY[2],ROI_XY[3]), (0,0,255), 1)
            cv2.drawMarker(img, REFERENCE_MARKER, (0,0,255))
            
        cv2.putText(img, mode_text, (1, 30), font, 0.4, (255, 0, 0), 1, cv2.LINE_AA)
        cv2.putText(img, fps_text, (1, 45), font, 0.4, (255, 0, 0), 1, cv2.LINE_AA)
        cv2.putText(img, block_count_text, (1, 60), font, 0.4, (255, 0, 0), 1, cv2.LINE_AA)
        cv2.drawMarker(img, ORIGIN_PIX, (0,255,0), thickness = 2)
        
        #
        #   key input mode/command
        #
        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            break
        elif key == ord('p'):
            mode_text = 'Mode: Pick'
            mode = MODE_PICK
        elif key == ord('s'):
            mode_text = 'Mode: Show'
            mode = MODE_SHOW
        elif key == ord('r'):
            roi_show = not roi_show
        else:
            pass

        # Display results
        #cv2.imshow('thresh', thresh)
        #cv2.imshow('erode', mask_erode)
        #cv2.imshow('dilate', mask)
        #cv2.imshow('Lab', img_Lab)
        cv2.imshow('blocks', img)
    
    cv2.destroyAllWindows()


###############################################################################
#
# get_rotation()
#
def get_rotation(box):
    """Return the rotation of a rectangle contour based on the slope of one of its long side."""
    
    x1 = 0
    x2 = 0
    
    # Take the first three vertices and calculate side lengths of the right angle triangle they form
    d1 = math.sqrt(pow((box[0][0]-box[1][0]),2)+pow((box[0][1]-box[1][1]),2))
    d2 = math.sqrt(pow((box[1][0]-box[2][0]),2)+pow((box[1][1]-box[2][1]),2))
    d3 = math.sqrt(pow((box[2][0]-box[0][0]),2)+pow((box[2][1]-box[0][1]),2))

    # Select vertices.
    # The shortest distance is the short side, the longest is the rectangle's diagonal.
    if (d1 > d2 and d1 < d3) or (d1 > d3 and d1 < d2):
        x1 = box[0][0]
        y1 = box[0][1]
        x2 = box[1][0]
        y2 = box[1][1]
    
    elif (d2 > d1 and d2 < d3) or (d2 > d3 and d2 < d1):
        x1 = box[1][0]
        y1 = box[1][1]
        x2 = box[2][0]
        y2 = box[2][1]
    
    elif (d3 > d1 and d3 < d2) or (d3 > d2 and d3 < d1):
        x1 = box[2][0]
        y1 = box[2][1]
        x2 = box[0][0]
        y2 = box[0][1]
    
    # Calculate the line rotation
    if x1 == x2:
        rotation = 90.0
    else:
        rotation = 57.29578049 * math.atan(float(y2-y1) / float(x2-x1))
        
    return rotation

###############################################################################
#
# move_arm()
#
def move_arm(arm_device, ik_model, x, y, z, grip_roration):
    """Calculate arm movement and send movement commands to robotic arm."""
    
    arm, boom, turret, grip_rotator, overrun = ik_model.get_positions(x,y,z, grip_roration)
    if overrun == True:
        return False

    motor_pos = arm_device.get_arm_positions(wait = True)
    arm_rate, boom_rate, turret_rate, grip_rotator_rate = ik_model.get_move_rates(arm, boom, turret, grip_rotator, motor_pos[1], motor_pos[2], motor_pos[0], motor_pos[3])
    arm_device.move_to(arm, arm_rate, boom, boom_rate, turret, turret_rate, grip_rotator, grip_rotator_rate)

    return True

###############################################################################
#
# Startup
#
if __name__ == '__main__':
    main()

