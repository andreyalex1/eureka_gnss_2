#!/usr/bin/env python3

#Developed by Andrei Smirnov. 2024
#MSU Rover Team. Voltbro. NIIMech 
from sensor_msgs.msg import NavSatFix
import numpy as np
import rclpy
from rclpy.node import Node
import serial


class ublox8_reader(Node):
    def __init__(self):
        super().__init__('ublox8_reader')
        self.pub = self.create_publisher(NavSatFix, "/gps", 10)
        self.time = 0.0
        self.lat = 0.0
        self.NS = 0.0
        self.long = 0.0
        self.EW = 0.0
        self.fix = 0.0
        self.alt = 0.0
        try:
            self.ser =  serial.Serial(port='/dev/gnss', baudrate = 9600, 
                                    parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, 
                                    bytesize=serial.EIGHTBITS, timeout=0.5)
        except serial.serialutil.SerialException:
            self.get_logger().warning("No USB Connection to UBLOX8!")
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.pipeline)
        self.get_logger().info("ublox8_reader Started!")
    def __del__(self):
        self.get_logger().info("ublox8_reader Killed!")
    def read(self):
        try:
            line = self.ser.readline()
            string = line.decode('ascii')
            words = string.split(',')
            for c in range(len(words)):
                if words[c] == '':
                    words[c] = '0.0'
            
            if words[0] == '$GNGGA':
                self.time = float(words[1])

                # Convert Latitude
                lat_degrees = int(float(words[2]) / 100)
                lat_minutes = float(words[2]) % 100
                self.lat = lat_degrees + (lat_minutes / 60)

                # Convert Longitude
                lon_degrees = int(float(words[4]) / 100)
                lon_minutes = float(words[4]) % 100
                self.long = lon_degrees + (lon_minutes / 60)

                self.NS = str(words[3])
                self.EW = str(words[5])
                self.fix = float(words[6])
                self.alt = float(words[9])

                # Apply direction correction
                if self.EW == 'W':
                    self.long *= -1
                if self.NS == 'S':
                    self.lat *= -1

        except serial.serialutil.SerialException:
            self.get_logger().warning("No USB Connection to UBLOX8!")
            try:
                self.ser =  serial.Serial(port='/dev/gnss', baudrate = 9600, 
                                  parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, 
                                  bytesize=serial.EIGHTBITS, timeout=0.5)
            except serial.serialutil.SerialException:
                None
    def send(self):
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
#        msg.status = self.fix
        msg.latitude = self.lat
        msg.longitude = self.long
        msg.altitude = self.alt
        self.pub.publish(msg)
    def print(self):
        print ("time = " + str(self.time))
        print ("lat = " + str(self.lat))
        print ("long = " + str(self.long))
        print ("alt = " + str(self.alt))
        print ("fix = " + str(self.fix))
        print("--------------------------------------")
    def pipeline(self):
        self.read()
        self.send()
        self.print()


def main(args=None):
    rclpy.init()
    usb = ublox8_reader()
    rclpy.spin(usb)

    
    usb.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()