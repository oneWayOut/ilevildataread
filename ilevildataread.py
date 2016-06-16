#!/usr/bin/python
# -*- coding: UTF-8 -*-

# iLevil_data_read.py have the function ofreading data from iLevil and save data
# It creates a file called flight_data in D:\
# It creates a txt with the name of local time in D:\flight_data
# The serial port that you will use should be input.The default value is 'COM5'，




import threading
import time
import serial;
#import binascii;


#import socket;




class ReadData:
    def __init__(self, Output=None, Port='COM5', Log=None):
        #Port='/dev/ttyUSB0'
        self.ser = None;


        self.logCppClock = False
        
        if self.logCppClock:
            import py_rt_module
            rt_module = py_rt_module.PyRT_Module()
            rt_module.start()  
            print "connection with RT_Timer done"
            
        self.port = Port;
        self.alive = False;
        self.stopFlag = False;

        self.waitEnd = None;
        self.thread_read = None;

        #self.sendport = '';
        #self.log = Log;
        #self.output = Output;

        self.Crc16Table=[]


        self.file_address=None
        self.file_name=''

        self.LocalTime_UTC='';

        self.data= [];
        self.messagepackage={};
        self.messagepackage_status=0

        self.messagepackage={'cppClockLine':'00:00:00','LocalTime_UTC':time.localtime(time.time()),'Time_Stamp_GPS':'00:00:00',\
        'Firmware_version':4.7,'Battery_Level':'100',\
        'Low_voltage_error':'Low_Votage','Excess_temperature':'T_normal',\
        'Roll_excess_error':'Roll_normal','Pitch_excess_error':'Pitch_normal','Yaw_excess_error':'Yaw_normal',\
        'Satellites':0,'GPS_signal_power':'0',\
        'Latitude':0,'Longitude':0,\
        'P_Alt_GPS':0,'GS_GPS':0,\
        'VSI_GPS':0,'T_H_status':'T',\
        'Track_Heading':0,'GPS_Alt':0,\
        'Roll':0,'Pitch':0,\
        'Yaw':0,'Inclination':0,\
        'Turn_Rate':0,'G_load':0,\
        'IAS':0,'P_Alt_AHRS':0,\
        'VSI_AHRS':0}

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++=++
    def Waiting(self):
        if not self.waitEnd is None:
            time.sleep(0.5)
            self.waitEnd.wait()

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    def SetStopEvent(self):
        if not self.waitEnd is None:
            self.waitEnd.set();

        self.alive = False;
        self.Stop();
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    def CreateCrc16Table(self):
        for i in range(256):
            crc_t=(i<<8)
            for bitctr in range(8):
                if (crc_t&0x8000):
                    crc_t=(crc_t<<1)^0x1021
                else:
                    crc_t=(crc_t<<1)^0
            self.Crc16Table.append(crc_t)

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    def Start(self):
        self.ser = serial.Serial();
        self.ser.port = self.port;
        self.ser.baudrate = 230400;
       

        self.ser.parity=serial.PARITY_NONE;
        self.ser.stopbits=serial.STOPBITS_ONE;
        self.ser.bytesize=serial.EIGHTBITS ;
        
        self.CreateCrc16Table()
        
    

        try:


            self.ser.open();
            #create a file in D:\flight_data\

            self.file_name='D:\\flight_data\\'+time.strftime('%Y-%m-%d %H%M%S',time.localtime(time.time()))+'.txt';
            self.file_address=open(self.file_name,'w')


            #Wirte Headline in the file
            self.file_address.writelines('%-20s'%('cppClockLine')+'%-20s'%('LocalTime_UTC')+'%-20s'%('Time_Stamp_GPS')+\
                                     '%-20s'%('Firmware_version')+'%-20s'%('Battery_Level')+\
                                     '%-30s'%('Low_voltage_error')+'%-20s'%('Excess_temperature')+\
                                     '%-20s'%('Roll_excess_error')+'%-20s'%('Pitch_excess_error')+'%-20s'%('Yaw_excess_error')+\
                                     '%-20s'%('Satellites')+'%-20s'%('GPS_signal_power')+\
                                     '%-20s'%('Latitude')+'%-20s'%('Longitude')+\
                                     '%-20s'%('P_Alt_GPS')+'%-20s'%('GS_GPS')+\
                                     '%-20s'%('VSI_GPS')+'%-20s'%('T_H_status')+\
                                     '%-20s'%('Track_Heading')+'%-20s'%('GPS_Alt')+\
                                     '%-10s'%('Roll')+'%-10s'%('Pitch')+\
                                     '%-10s'%('Yaw')+'%-20s'%('Inclination')+\
                                     '%-10s'%('Turn_Rate')+'%-10s'%('G_load')+\
                                     '%-10s'%('IAS')+'%-20s'%('P_Alt_AHRS')+\
                                     '%-20s'%('VSI_AHRS')+'\n')      
        except Exception,ex:
            print str(ex)
            if self.ser.isOpen():
                self.ser.close();

            self.ser = None;
            return False;



        if self.ser.isOpen():
            self.waitEnd = threading.Event();
            self.alive = True;
            self.thread_read = threading.Thread(target=self.FirstReader);
            self.thread_read.setDaemon(1);
            self.thread_read.start();
          
            return True;
        else:
            return False;

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    def FindCE(self,dat_message):

    #find control_escape_character 0x7D,del 7D and the character after XORed by 0x20

        if chr(int('7D',16)) in dat_message:
            #print 'findCE find 7D'
            index_7D=dat_message.index(chr(int('7D',16)))

            character_7D1=ord(dat_message[index_7D+1])^0x20


            if character_7D1==0x7D or character_7D1==0x7E:


                dat_message[index_7D+1]=chr(character_7D1)
                del(dat_message[index_7D])
            else:
                dat_message=[];

        else:
            pass
            #print 'findCE not find 7D '

        return dat_message

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    def MatchFCS(self,d_message):
        # FCS is CRC-CCITT
        #Caculate FCS on the clear message, match with the FCS character in the message
        #if equal,save the message,otherwise discard it

        FCS=(ord(d_message[-1])<<8)+ord(d_message[-2])
        del d_message[-2:]

        crc=0
        for i in range(len(d_message)):
            crc=(self.Crc16Table[crc>>8])^(crc<<8)^(ord(d_message[i]))
            if crc >=pow(2,16):
                crc=crc-((crc>>16)<<16)

        if crc==FCS:
            #print 'matchFCS match'
            return True
        else:
            #print 'matchFCS not match'
            return False




#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    def MatchmessageID(self,i_message):


        if ord(i_message[0])==0:
            num_message=len(i_message);
            for i in range(num_message):
                i_message[i]=ord(i_message[i])

            Time_Stamp_GPS_int=(i_message[4]<<8)+i_message[3]+((i_message[2]&0x80)<<9)
            time_h=Time_Stamp_GPS_int/3600
            time_m=int((Time_Stamp_GPS_int/3600.0-Time_Stamp_GPS_int/3600)*60)
            time_s=int(((Time_Stamp_GPS_int/3600.0-Time_Stamp_GPS_int/3600)*60-int((Time_Stamp_GPS_int/3600.0-Time_Stamp_GPS_int/3600)*60))*60)

            self.messagepackage['Time_Stamp_GPS']=str(time_h)+':'+str(time_m)+':'+str(time_s)


            self.messagepackage_status=1

        # GPS data
        elif ord(i_message[0])==10:

            num_message=len(i_message);
            for i in range(num_message):
                i_message[i]=ord(i_message[i])

            # Latitude
            tempInt= (i_message[5]<<16)+(i_message[6]<<8)+i_message[7]

            if tempInt & 0x800000:
                self.messagepackage['Latitude']=-((tempInt^0xFFFFFF)+1)*90.0/4194304;
            else:
                self.messagepackage['Latitude']=tempInt*90.0/4194304;

             #Longitude
            tempInt= (i_message[8]<<16)+(i_message[9]<<8)+i_message[10]

            if tempInt & 0x800000:
                self.messagepackage['Longitude']=-((tempInt^0xFFFFFF)+1)*90.0/4194304;
            else:
                self.messagepackage['Longitude']=tempInt*90.0/4194304-360;


            #******Pressure Altitude******ft
            self.messagepackage['P_Alt_GPS']=((i_message[11]<<4)+(i_message[12]>>4))*25-1000;

            #Horizontal velocity kt

            self.messagepackage['GS_GPS']=(i_message[14]<<4)+(i_message[14]>>4);

            #Vertical velocity GPS fpm
            if (i_message[15]&8) == 8:
                self.messagepackage['VSI_GPS']=-(((i_message[15]-((i_message[15]>>3)<<3))<<8)+i_message[16])*64;
            else:
                self.messagepackage['VSI_GPS']=(((i_message[15]-((i_message[15]>>3)<<3))<<8)+i_message[16])*64;

            #Track/Heading 0=North,128=South
            self.messagepackage['Track_Heading']=i_message[17]*360.0/256;

            if (i_message[12]-((i_message[12]>>2)<<2))==1:
                self.messagepackage['T_H_status']='True_Track';
            elif (i_message[12]-((i_message[12]>>2)<<2))==2:
                self.messagepackage['T_H_status']='Heading_Mag';
            elif (i_message[12]-((i_message[12]>>2)<<2))==3:
                self.messagepackage['T_H_status']='Heading_True';

            self.messagepackage_status=1

       # Ownship GEO Altitude
        elif ord(i_message[0])==11:
            i_message[1]=ord(i_message[1])
            i_message[2]=ord(i_message[2])

            if (i_message[1]&128)==128:
                self.messagepackage['GPS_Alt']=-((i_message[1]<<8)+i_message[2])*5
            else:
                self.messagepackage['GPS_Alt']=((i_message[1]<<8)+i_message[2])*5

            self.messagepackage_status=1


        elif (i_message[0])=='L' and (i_message[1]=='E'):

            num_message=len(i_message);
            for i in range(num_message):
                i_message[i]=ord(i_message[i])

            # status of the equipment
            if (i_message[2]==0) and (i_message[3]==2):

                self.messagepackage['Firmware_version']=i_message[4]*0.1;
                self.messagepackage['Battery_Level']=i_message[5];
                #print i_message[4]
                #print i_message[5]
                #print i_message[6]
                #print i_message[7]
                


                if (i_message[6]&8)==8:
                    self.messagepackage['Low_voltage_error']='Low_Votage'
                else:
                    self.messagepackage['Low_voltage_error']='Voltage_normal'

                if (i_message[6]&16)==16:
                    self.messagepackage['Excess_temperature']='T>60'
                else:
                    self.messagepackage['Excess_temperature']='T_normal'

                if (i_message[6]&32)==32:
                    self.messagepackage['Roll_excess_error']='Roll_error'
                else:
                    self.messagepackage['Roll_excess_error']='Roll_normal'

                if (i_message[6]&64)==64:
                    self.messagepackage['Pitch_excess_error']='Pitch_error'
                else:
                    self.messagepackage['Pitch_excess_error']='Pitch_normal'

                if (i_message[6]&128)==128:
                    self.messagepackage['Yaw_excess_error']='Yaw_error'
                else:
                    self.messagepackage['Yaw_excess_error']='Yaw_normal'
                    

                if (i_message[8]&1)==1:
                    self.messagepackage['WAAS_GPS_Status']='WAAS_enabled'
                else:
                    self.messagepackage['WAAS_GPS_Status']='No_WAS'

                self.messagepackage_status=1

            # GPS status
            if i_message[2]==7:

                #Satellites
                self.messagepackage['Satellites']=i_message[5];

                #Power
                self.messagepackage['GPS_signal_power']=((i_message[6]<<8)+i_message[7])*0.1


                # Altitude variation

                self.messagepackage['Altitue_vvariation']=(i_message[11]<<8)+i_message[12]

                self.messagepackage_status=1


            # AHRS data
            if (i_message[2])==1 and (i_message[3]==1):


                #Roll
                if (i_message[4]&128) == 128:
                    self.messagepackage['Roll']=-(((i_message[4]-128)<<8)+i_message[5])*0.1;
                else:
                    self.messagepackage['Roll']=((i_message[4]<<8)+i_message[5])*0.1;

                #Pitch
                if (i_message[6]&128) == 128:
                    self.messagepackage['Pitch']=-(((i_message[6]-128)<<8)+i_message[7])*0.1;
                else:
                    self.messagepackage['Pitch']=((i_message[6]<<8)+i_message[7])*0.1;

                #Yaw
                if (i_message[8]&128) == 128:
                    self.messagepackage['Yaw']=-(((i_message[8]-128)<<8)+i_message[9])*0.1;
                else:
                    self.messagepackage['Yaw']=((i_message[8]<<8)+i_message[9])*0.1;

                #Inclination
                if (i_message[10]&128) == 128:
                    self.messagepackage['Inclination']=-(((i_message[10]-128)<<8)+i_message[11])*0.1;
                else:
                    self.messagepackage['Inclination']=((i_message[10]<<8)+i_message[11])*0.1;

                #Turn Coordinator
                if (i_message[12]&128) == 128:
                    self.messagepackage['Turn_Rate']=-(((i_message[12]-128)<<8)+i_message[13])*0.1;
                else:
                    self.messagepackage['Turn_Rate']=((i_message[12]<<8)+i_message[13])*0.1;

                #G load
                if (i_message[14]&128) == 128:
                    self.messagepackage['G_load']=-(((i_message[14]-128)<<8)+i_message[15])*0.1;
                else:
                    self.messagepackage['G_load']=((i_message[14]<<8)+i_message[15])*0.1;

                #Indicated Airspeed
                if (i_message[16]&128) == 128:
                    self.messagepackage['IAS']=-(((i_message[16]-128)<<8)+i_message[17])*0.1;
                else:
                    self.messagepackage['IAS']=((i_message[16]<<8)+i_message[17])*0.1;

                #Pressure Altitude
                self.messagepackage['P_Alt_AHRS']=(i_message[18]<<8)+i_message[19]-5000;

                #Vertial speed_AHRS
                if (i_message[20]&128) == 128:
                    self.messagepackage['VSI_AHRS']=-(((i_message[20]-128)<<8)+i_message[21])*0.1;
                else:
                    self.messagepackage['VSI_AHRS']=((i_message[20]<<8)+i_message[21])*0.1;

                self.messagepackage_status=1

        else: self.messagepackage_status=0

        #print 'match message ID normal'

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    def SaveData(self):
        if self.file_address.closed is True:
            self.file_address=open(self.file_name,'a+');

        cppClockLine = ""
        if self.logCppClock:
            cppTime = rt_module.get_time() # Get the real time value
            cppClockLine = str(cppTime)+","

        self.file_address.writelines('%-20s'% cppClockLine+'%-20s'%(self.LocalTime_UTC)+'%-20s'%(self.messagepackage['Time_Stamp_GPS'])+\
        '%-20.1f'%(float(self.messagepackage['Firmware_version']))+'%-20.1f'%(float(self.messagepackage['Battery_Level']))+\
        '%-30s'%(self.messagepackage['Low_voltage_error'])+'%-20s'%(self.messagepackage['Excess_temperature'])+\
        '%-20s'%(self.messagepackage['Roll_excess_error'])+'%-20s'%(self.messagepackage['Pitch_excess_error'])+'%-20s'%(self.messagepackage['Yaw_excess_error'])+\
        '%-20d'%(int(self.messagepackage['Satellites']))+'%-20.1f'%(float(self.messagepackage['GPS_signal_power']))+\
        '%-20.8f'%(float(self.messagepackage['Latitude']))+'%-20.8f'%(float(self.messagepackage['Longitude']))+\
        '%-20d'%(int(self.messagepackage['P_Alt_GPS']))+'%-20d'%(int(self.messagepackage['GS_GPS']))+\
        '%-20d'%(int(self.messagepackage['VSI_GPS']))+'%-20s'%(self.messagepackage['T_H_status'])+\
        '%-20.1f'%(float(self.messagepackage['Track_Heading']))+'%-20d'%(int(self.messagepackage['GPS_Alt']))+\
        '%-10.1f'%(float(self.messagepackage['Roll']))+'%-10.1f'%(float(self.messagepackage['Pitch']))+\
        '%-10.1f'%(float(self.messagepackage['Yaw']))+'%-20.1f'%(float(self.messagepackage['Inclination']))+\
        '%-10.1f'%(float(self.messagepackage['Turn_Rate']))+'%-10.2f'%(float(self.messagepackage['G_load']))+\
        '%-10.1f'%(float(self.messagepackage['IAS']))+'%-20d'%(int(self.messagepackage['P_Alt_AHRS']))+\
        '%-20d'%(int(self.messagepackage['VSI_AHRS']))+'\n')

        #print 'savedata normal'
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    def SendData(self):

        
        print self.messagepackage['Roll']
        print self.messagepackage['Pitch']
        print self.messagepackage['Yaw']
        #print self.messagepackage['Time_Stamp_GPS']
        pass



#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++



#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    def FirstReader(self):

        time.sleep(2)


        while (self.alive and (not self.stopFlag)):


            try:
                time.sleep(1);
                n = self.ser.inWaiting(); #inWaiting()：ruturn the length of the bytes in the buffer

                #receive the bytes in the buffer, ascii. All the bytes means one frame
                if n:
                    self.data = list(self.ser.read(n));
                else:
                    continue

                num=len(self.data)


                self.LocalTime_UTC=time.strftime('%H:%M:%S',time.localtime(time.time()))
                print self.LocalTime_UTC


                # Process the frame of data received
                #+++ find the first flag 7E and delete all the bytes before

                while ord(self.data[0])!=0x7E:
                    del self.data[0]

                #+++ find the last flag 7E and delete all the bytes after

                while ord(self.data[-1])!=0x7E:
                    del self.data[-1]


                #If the self.data contain at least three bytes,first one and the last is 7E
                # the second one may be message or 7E
                # otherwise break,

                if (len(self.data)>2) and (ord(self.data[1])!=0x7E):


                    pass
                else:
                    self.messagepackage_status=0
                    continue

                #print '7D and 7E'
                #if the data of the frame still have bytes , process byte by byte

                while (self.data!=[]):

                    # if 7E exist,delete 7E and deal with the data

                    del self.data[0]
                    if self.data==[]:
                        continue

                    data_message=[];

                    #save message before sencond 7E,process the message and conninue the next message

                    while ord(self.data[0])!=0x7E:

                        #data_message ,bytes between two flag 7E
                        #data_message.append(binascii.hexlify(self.data[0]))
                        data_message.append(self.data[0])
                        del(self.data[0])



                    if (data_message!=[]):
                        # find Contol escape character 7D and process
                        data_message=self.FindCE(data_message);
                        if data_message==[]:
                            continue
                        else:
                            # match FCS,data_massage include FCS     
                            if self.MatchFCS(data_message):
                                self.MatchmessageID(data_message)
                            else:
                                continue

                # If a message
                if self.messagepackage_status:
                    self.SaveData();
                    #self.SendDate();
                else: pass


            except Exception, ex:
                print str(ex)
                self.SetStopEvent()



#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    def Stop(self):

        if not self.waitEnd is None:
            self.waitEnd.set();

        if not self.thread_read is None:
            self.thread_read._Thread__stop();

        if self.alive:
            self.alive = False;
            self.ser.close();

        if not self.file_address is None:
            self.file_address.close()





#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++




