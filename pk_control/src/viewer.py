#!/usr/bin/env python3

# based on Aditya's code
# provides a camera feed display from Jackal, using the tkinter UI framework
# note: you will need to run this: sudo apt-get install python3-pil python3-pil.imagetk

import rospy
import numpy as np
import cv2
from tkinter import *
from PIL import Image, ImageTk
from std_msgs.msg import String
from sensor_msgs.msg import Joy
from sensor_msgs.msg import CompressedImage

VERBOSE = False


class Control():
    def __init__(self, window, camera="flir", img_scale=0.8):
        ############### Tkinter Window Properties and Setup ########
        self.img_scale_factor = img_scale  # display at 80% of the incoming size
        self.window = window
        self.window.label = Label(self.window)
        self.window.label.pack(anchor=CENTER)
        self.imgtk = None
        rospy.loginfo("called init")
        if camera=="flir":
            rospy.loginfo("using flir")
            self.flir_image = rospy.Subscriber("/camera/image_color/compressed", CompressedImage, self.update_image,
                                               queue_size=1)
        else:
            rospy.loginfo("using axis")
            self.axis_image = rospy.Subscriber("/axis/image_raw/compressed", CompressedImage, self.update_image,
                                                queue_size=1)

    def update_image(self, ros_data):
        #rospy.loginfo("update img")
        '''
        Function to update image from a camera for the corresponding tkinter window
        '''
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        image_np = cv2.cvtColor(image_np, cv2.COLOR_BGR2RGB)

        new_width = int(image_np.shape[1] * self.img_scale_factor)
        new_height = int(image_np.shape[0] * self.img_scale_factor)
        new_dims = (new_width, new_height)
        image_np = cv2.resize(image_np, new_dims)

        img = Image.fromarray(image_np)
        self.imgtk = ImageTk.PhotoImage(image=img)
        self.window.label.config(image=self.imgtk)
        self.window.label.image = self.imgtk

def main():
    #rospy.loginfo("Do main")
    rospy.init_node("viewer", anonymous=True)
    rospy.loginfo("viewer node started...")
    window = Tk()
    window.geometry("800x600")#"700x550")
    window.title("Jackal's Forward Camera")
    controller = Control(window)
    #rospy.loginfo("Do main")

    window2 = Toplevel(window)
    window2.geometry("800x600")#"700x550")
    window2.title("Jackal's Rear Camera")
    controller2 = Control(window2, "axis")

    window3 = Toplevel(window2)
    window3.geometry("200x300")
    window3.title("Rear Camera Controls")
    btn1 = Button(window3,text='Left',command=rospy.loginfo("Clicked!")).pack(expand=True)

    try:
        window.mainloop()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()

# for the future:
# - Want a third window with Axis camera controls on it (pan,tilt,zoom?,focus?,autofocus?,brightness?,iris?)
# - Want to be able to send velocity commands to the axis camera (thus proportional to joystick)
# could do this by: assume publishRate == dt
# setNewPan= getPan + (velIn*publishRate)
# and of course velIn = joy*someConst
