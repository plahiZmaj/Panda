from __future__ import print_function
import cv2
from take_photo import image_converter
import time
import sys


import rospy

def edit_img():
    ic = image_converter()
    image = ic.take_image()
    ic.save_image(image)
    #color = int(image[5,5])

    # Window name in which image is displayed
    window_name = 'Image'
    
    # Start coordinate, here (5, 5)
    # represents the top left corner of rectangle
    tocke = {
     "tocka1": (505, 260),
     "tocka2": (730, 260),
     "tocka3": (495, 380),
     "tocka4": (750, 380),
     "tocka5": (490, 535),
     "tocka6": (750, 535)
     }

    tocke_key_values = ["tocka1","tocka2","tocka3","tocka4","tocka5","tocka6"]
    point_has_item=[0,0,0,0,0,0]

    for num,test in enumerate(tocke):
        ii = tocke[tocke_key_values[num]]
        print(ii)
        start_point = (ii[0], ii[1])
        end_point = (start_point[0]+30, start_point[1]+30)
         # Line thickness of 2 px
        thickness = 4
        value_counter = 0
        for i in range(30):
            for j in range(30):
                (b, g, r) = image[ii[1]+i,ii[0]+j]
                #print("Pixel at ({}, {}) - Red: {}, Green: {}, Blue: {}".format(ii[0],ii[1],r, g, b))
                if (r<10):
                    pass#value_counter += value_counter
                else:
                    value_counter = value_counter+1            
        if(value_counter<650):
            point_has_item[num] = 0
            color = (0, 0, 255)#b g r
        else:
            point_has_item[num] = 1
            color = (0, 255, 0)

        # Using cv2.rectangle() method
        # Draw a rectangle with blue line borders of thickness of 2 px
        image = cv2.rectangle(image, start_point, end_point, color, thickness)
    

    print(point_has_item)

    #start_point = (490, 535)
    #(b, g, r) = image[start_point[1],start_point[0]]
    #print("Pixel at (50, 20) - Red: {}, Green: {}, Blue: {}".format(r, g, b))

    # Ending coordinate, here (220, 220)
    # represents the bottom right corner of rectangle
    #end_point = (start_point[0]+30, start_point[1]+30)
    
    # Blue color in BGR
    #color = (255, 0, 0)
    
    # Line thickness of 2 px
    #thickness = 1
    
    # Using cv2.rectangle() method
    # Draw a rectangle with blue line borders of thickness of 2 px
    #image = cv2.rectangle(image, start_point, end_point, color, thickness)
    
    # Displaying the image 
    cv2.imshow(window_name, image) 
    cv2.waitKey(2000)

    return point_has_item

if __name__ == "__main__":
    rospy.init_node('edit_img')
    edit_img()


