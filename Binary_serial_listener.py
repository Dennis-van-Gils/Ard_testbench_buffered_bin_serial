#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Nov  8 14:19:05 2018

@author: Dennis_van_Gils
"""

import serial
import struct

import matplotlib.pyplot as plt
import numpy as np
import time as Time

SOM = bytes([0x00, 0x00, 0x00, 0x00, 0xee]) # Start of message
EOM = bytes([0x00, 0x00, 0x00, 0x00, 0xff]) # End of message

fn_log = "log.txt"

if __name__ == "__main__":
    try:
        ser.close()
    except:
        pass
    ser = serial.Serial("COM6", baudrate=1500000)    
    #ser = serial.Serial("COM4")
    f_log = open(fn_log, 'w')

    #while 1:
    full_time = np.array([], dtype=int)
    full_wave = np.array([], dtype=float)

    counter = 0
    N_count = 50
    time_start = Time.time()
    while counter < N_count:
        counter += 1
        ans_bytes = ser.read_until(EOM)
        ans_bytes = ans_bytes[:-5]      # Remove EOM
        
        if (ans_bytes[:5] == SOM):
            ans_bytes = ans_bytes[5:]      # Remove SOM
            N_samples = int(len(ans_bytes) / struct.calcsize('If'))
            time_bytes = ans_bytes[:N_samples * struct.calcsize('I')]
            wave_bytes = ans_bytes[N_samples * struct.calcsize('I'):]
            try:
                time = struct.unpack('<' + 'I'*N_samples, time_bytes)
                wave = struct.unpack('<' + 'f'*N_samples, wave_bytes)
            except Exception as err:
                ser.close()
                f_log.close()
                raise(err)
                
            full_time = np.append(full_time, time)
            full_wave = np.append(full_wave, wave)

            print("%d: %d" % (counter, N_samples))
            #print("%10i\t%7.4f" % (time[0], wave[0]))
            for i in range(N_samples):
                f_log.write("%i\t%.6f\n" % (time[i], wave[i]))
    time_end = Time.time()
    
    fig, ax = plt.subplots()
    ax.plot(full_time, full_wave, 'x-')
    ax.set(xlabel='time (usec)', ylabel='y')
    ax.grid()
    plt.show()
    
    dt = np.diff(full_time)
    Fs = 1/np.mean(dt)*1e6
    print("Fs = %.2f Hz    dt_min = %d us    dt_max = %d us" % 
          (Fs, np.min(dt), np.max(dt)))
    print("%.2f buffers read per sec" % (N_count/(time_end - time_start)))
    
    f_log.close()
    ser.close()