from Tkinter import *
import os
class Application(Frame):
    def Start_demo(self):
        os.system('/data/ros/ur_ws_yue/src/uwb_uart/Demo/demo.sh')
        print ("Lets do This!")

    def createWidgets(self):
        self.QUIT = Button(self)
        self.QUIT["text"] = "QUIT"
        self.QUIT["fg"]   = "red"
        self.QUIT["font"] = ('Times 10 bold')
        self.QUIT["command"] =  self.quit
        self.QUIT.pack({"side": "left"})

        self.hi_demo = Button(self)#width = 30,height = 2)
        self.hi_demo["text"] = "START"
        self.hi_demo["width"] = 100
        self.hi_demo["height"] = 50
        self.hi_demo["font"] =('Times 300 bold')
        # self.hi_demo["bitmap"]BitmapImage(file = "/data/ros/ur_ws_yue/src/uwb_uart/Demo/1.jpg")
        self.hi_demo["command"] = self.Start_demo

        self.hi_demo.pack({"side": "left"})

    def __init__(self, master=None):
        Frame.__init__(self, master)
        self.pack()
        self.createWidgets()

root = Tk()
root.title("DEMO")
app = Application(master=root)
app.mainloop()
root.destroy()
