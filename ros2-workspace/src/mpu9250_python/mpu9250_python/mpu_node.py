#!/usr/bin/env python3

# https://github.com/flynneva/bno055/blob/main/bno055/sensor/SensorService.py
# https://github.com/kriswiner/MPU6050/wiki/Affordable-9-DoF-Sensor-Fusion

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data
from rclpy.parameter import Parameter

import os
from sensor_msgs.msg import Imu, MagneticField, Temperature
from std_msgs.msg import String
from geometry_msgs.msg import Quaternion
#from mpu9250_i2c import *
import time
from mpu9250_jmdev.registers import *
from mpu9250_jmdev.mpu_9250 import MPU9250
#from tf.transformations import quaternion_about_axis
debug = False

class MpuPublisher(Node):

    def __init__(self):
        super().__init__('mpu_9250_python')
        self.declare_parameters(
        namespace='',
        parameters=[
          ('my_str', None),
          ('my_int', None),
          ('my_double_array', None)
        ] 
        )
        self.declare_parameter('frame_id', 'imu_frame')

        self.mpu = MPU9250(
          address_ak=AK8963_ADDRESS, 
          address_mpu_master=MPU9050_ADDRESS_68, # In 0x68 Address
          address_mpu_slave=None, 
          bus=1, 
          gfs=GFS_1000, 
          afs=AFS_8G, 
          mfs=AK8963_BIT_16, 
          mode=AK8963_MODE_C100HZ)

        self.mpu.configure() # Apply the settings to the registers.


        # todo make this parameters, and do some calibration service or something
        #self.mpu.abias = [0.024116330030487805, 0.09404177782012195, -0.20670374428353655]
        #self.mpu.gbias =  [4.080423494664634, -6.630874261623475, -5.128627870141006]
        #self.mpu.magScale =  [1.2469135802469136, 0.8632478632478632, 0.9619047619047618]
        #self.mpu.mbias =  [17.125546588827838, 24.838324175824177, 17.700995879120878]

        #self.mpu.abias = [-0.08004239710365854, 0.458740234375, 0.2116996951219512]
        #self.mpu.gbias = [0.8958025676448171, 0.45292551924542684, 0.866773651867378]
        #self.mpu.magScale = [1.0104166666666667, 0.9797979797979799, 1.0104166666666667]
        #self.mpu.mbias = [2.6989010989010986, 2.7832417582417586, 2.6989010989010986]



        #self.declare_parameter('variance_acc', value=registers.DEFAULT_VARIANCE_ACC)
        #self.declare_parameter('variance_angular_vel', value=registers.DEFAULT_VARIANCE_ANGULAR_VEL)
        #self.declare_parameter('variance_orientation', value=registers.DEFAULT_VARIANCE_ORIENTATION)
        #self.declare_parameter('variance_mag', value=registers.DEFAULT_VARIANCE_MAG)

        time.sleep(1) # delay necessary to allow mpu9250 to settle

        QoSProf = QoSProfile(depth=10)
        prefix = "imu/"
        # create topic publishers:
        self.pub_imu_raw = self.create_publisher(Imu, prefix + 'data_raw', QoSProf)
        #self.pub_imu = self.create_publisher(Imu, prefix + 'imu', QoSProf)
        self.pub_mag = self.create_publisher(MagneticField, prefix + 'mag', QoSProf)
        self.pub_temp = self.create_publisher(Temperature, prefix + 'temp', QoSProf)
        #self.pub_calib_status = node.create_publisher(String, prefix + 'calib_status', QoSProf)
        #self.srv = self.node.create_service(Trigger, prefix + 'calibration_request', self.calibration_request_callback)

        timer_period = 0.001  # seconds
        self.timer = self.create_timer(timer_period, self.imu_callback)
        self.i = 0
       

    def imu_callback(self):
        """
        Callback function.
        """
        if debug:

            print("Accelerometer", self.mpu.readAccelerometerMaster())
            print("Gyroscope", self.mpu.readGyroscopeMaster())
            print("Magnetometer", self.mpu.readMagnetometerMaster())
            print("Temperature", self.mpu.readTemperatureMaster())
        #a = self.mpu.readAccelerometerMaster()
        #g = self.mpu.readGyroscopeMaster()
        #m = self.mpu.readMagnetometerMaster()
        #t = self.mpu.readTemperatureMaster()

#['timestamp', 'master_acc_x', 'master_acc_y', 'master_acc_z', 'master_gyro_x', 'master_gyro_y', 'master_gyro_z', 'slave_acc_x', 'slave_acc_y', 'slave_acc_z', 'slave_gyro_x', 'slave_gyro_y', 'slave_gyro_z', 'mag_x', 'mag_y', 'mag_z', 'master_temp', 'slave_temp']
#[1658436898.8061285, 0.020751953125, 0.097900390625, 0.786376953125, 4.119873046875, -6.561279296875, -5.126953125, 0, 0, 0, 0, 0, 0, 16.68187957875458, 25.639560439560437, 17.102411477411476, 43.90711953754455, 0]
        labels = self.mpu.getAllDataLabels() # return labels with data description for each array position
        d = self.mpu.getAllData() # returns a array with data from all sensors

        tsmpu = d[0]
        ax = d[1]
        ay = d[2]
        az = d[3]
        wx = d[4]
        wy = d[5]
        wz = d[6]
        mx = d[13]
        my = d[14]
        mz = d[15]
        temp = d[16]
                
        #try:
        #    ax,ay,az,wx,wy,wz = mpu6050_conv() # read and convert mpu6050 data
        #    mx,my,mz = AK8963_conv() # read and convert AK8963 magnetometer data
        #except:
        #    print("failure")
       
        if debug:
            print('{}'.format('-'*30))
            print('accel [g]: x = {0:2.2f}, y = {1:2.2f}, z {2:2.2f}= '.format(ax,ay,az))
            print('gyro [dps]:  x = {0:2.2f}, y = {1:2.2f}, z = {2:2.2f}'.format(wx,wy,wz))
            print('mag [uT]:   x = {0:2.2f}, y = {1:2.2f}, z = {2:2.2f}'.format(mx,my,mz))
            print('temperature: ', temp)
            print('{}'.format('-'*30))
    
        stamp = self.get_clock().now().to_msg()
        frame_id = self.get_parameter("frame_id").value

        imu_raw_msg = Imu()
        #imu_msg = Imu()
        mag_msg = MagneticField()
        temp_msg = Temperature() 
        temp_msg.header.stamp = stamp
        temp_msg.header.frame_id = frame_id
        # temp_msg.header.seq = seq
        temp_msg.temperature = temp
        self.pub_temp.publish(temp_msg)

        imu_raw_msg.header.stamp = stamp
        imu_raw_msg.header.frame_id = frame_id

        ## Calculate a quaternion representing the orientation
        #accel = accel_x, accel_y, accel_z
        #ref = np.array([0, 0, 1])
        #acceln = accel / np.linalg.norm(accel)
        #axis = np.cross(acceln, ref)
        #angle = np.arccos(np.dot(acceln, ref))
        #orientation = quaternion_about_axis(angle, axis)

        #imu_raw_msg.orientation_covariance = [
        #    self.param.variance_orientation.value[0], 0.0, 0.0,
        #    0.0, self.param.variance_orientation.value[1], 0.0,
        #    0.0, 0.0, self.param.variance_orientation.value[2]
        #]       

        imu_raw_msg.linear_acceleration.x = ax
        imu_raw_msg.linear_acceleration.y = ay
        imu_raw_msg.linear_acceleration.z = az
        #imu_raw_msg.linear_acceleration_covariance = [
        #    self.param.variance_acc.value[0], 0.0, 0.0,
        #    0.0, self.param.variance_acc.value[1], 0.0,
        #    0.0, 0.0, self.param.variance_acc.value[2]
        #]       
        imu_raw_msg.angular_velocity.x = wx
        imu_raw_msg.angular_velocity.y = wy
        imu_raw_msg.angular_velocity.z = wz
        self.pub_imu_raw.publish(imu_raw_msg) 

        # Publish magnetometer data
        mag_msg.header.stamp = stamp
        mag_msg.header.frame_id = frame_id
        # mag_msg.header.seq = seq
        mag_msg.magnetic_field.x = mx
        mag_msg.magnetic_field.y = my
        mag_msg.magnetic_field.z = mz
        #mag_msg.magnetic_field_covariance = [
        #    self.param.variance_mag.value[0], 0.0, 0.0,
        #    0.0, self.param.variance_mag.value[1], 0.0,
        #    0.0, 0.0, self.param.variance_mag.value[2]
        #]
        self.pub_mag.publish(mag_msg)
 
def main(args=None):
    rclpy.init(args=args)

    publisher = MpuPublisher()

    rclpy.spin(publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
