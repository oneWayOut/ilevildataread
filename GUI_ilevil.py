#!/usr/bin/env python

import Tkinter
from Tkinter import *
import time
import threading
import random
import Queue
import pdb


import ilevildataread

class GuiPart:
    def __init__(self, master, queue, startCommand, endCommand):
        self.queue = queue

        # Set up the GUI
        titleLabel=Label(master,text='iLevil data processing')
        titleLabel.grid(row=1,column=1)
        consoleStart = Tkinter.Button(master, text='Record on', command=startCommand)
        consoleStart.grid(row=2,column=1,sticky=W)

        consoleEnd = Tkinter.Button(master, text='Record off', command=endCommand)
        consoleEnd.grid(row=3,column=1,sticky=E)

        # Add more GUI stuff here depending on your specific needs

    def processIncoming(self):
        """Handle all messages currently in the queue, if any."""
        
        while self.queue.qsize(  ) :
            pass

class ThreadedClient:
    """
    Launch the main part of the GUI and the worker thread. periodicCall and
    endApplication could reside in the GUI part, but putting them here


    means that you have all the thread controls in a single place.
    """
    def __init__(self, master):
        """
        Start the GUI and the asynchronous threads. We are in the main


        (original) thread of the application, which will later be used by
        the GUI as well. We spawn a new thread for the worker (I/O).
        """
        self.master = master


        self.rdd = ilevildataread.ReadData()

        # Create the queue
        self.queue = Queue.Queue(  )

        # Set up the GUI part
        self.gui = GuiPart(master, self.queue, self.startApplication, self.endApplication)

        # Set up the thread to do asynchronous I/O
        # More threads can also be created and used, if necessary
        self.running = 0
        self.thread1 = threading.Thread(target=self.workerThread1)


        self.thread1.start(  )

        # Start the periodic call in the GUI to check if the queue contains
        # anything
        self.periodicCall(  )

    def periodicCall(self):
        """
        Check every 200 ms if there is something new in the queue.
        """
        
        if self.running:            
            self.gui.processIncoming(  )

        self.master.after(200, self.periodicCall)

    def workerThread1(self):
        """
        This is where we handle the asynchronous I/O. For example, it may be
        a 'select(  )'. One important thing to remember is that the thread has
        to yield control pretty regularly, by select or otherwise.
        """
        while 1:
            if self.running :
                self.rdd.SendData()
                time.sleep(1)
            else:
                time.sleep(1)


    def endApplication(self):
        self.running = 0
        
        
        self.rdd.Stop()

    def startApplication(self):
        #pdb.set_trace()
        self.running = 1
        #self.gui.consoleStart.set(Color='green')
        self.rdd.Start()
        
rand = random.Random(  )
root = Tkinter.Tk(  )

client = ThreadedClient(root)


def on_closing():   
    client.thread1._Thread__stop()
    root.destroy()
    

root.protocol("WM_DELETE_WINDOW", on_closing)



root.mainloop(  )


