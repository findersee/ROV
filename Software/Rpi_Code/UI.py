import tkinter
from tkinter import *
import cv2
from PIL import ImageTk, Image




def togglePic():
    #print(button.cget('image'))
    if button.cget('image') == 'pyimage1':
        button.configure(image=btnOn)
    else:
        button.configure(image=btnOff)
    #button.config(image=btnOn)
    #button.pack()

#def main():
window = Tk() 
app = Frame(window,bg="blue")
app.grid
app.pack()
lmain=Label(app, text="Toimiikohan tämä")
lmain.grid
lmain.pack()


cap = cv2.VideoCapture('rtsp://admin:Kiv2Camera@192.168.0.10:554/videoMain')


def video_stream():
    _, frame = cap.read()
    cv2image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGBA)
    cv2.imshow('Test',cv2image)
    img = Image.fromarray(cv2image)
    print(img)
    #imgtk = Image.PhotoImage(image=img)
    imgtk = PhotoImage(img)
    lmain.imgtk = imgtk
    lmain.configure(image=imgtk)
    #print('Test')
    lmain.after(1, video_stream) 


#greeting = Label(text="Haista, Home...")  
#greeting.pack()
#button = Button(window,command=togglePic)
#btnOff = PhotoImage(file="Img/Bulb_off.png")
#btnOn = PhotoImage(file="Img/Bulb_on.png")
#button.config(image=btnOff,borderwidth=0)
#button.pack() 
video_stream()
window.mainloop()

#while(1):
#    ref, frame = cap.read()
#    cv2.imshow('VIDEO',frame)
#    cv2.waitKey(1)

    
#if __name__ == "__main__":
#    main()