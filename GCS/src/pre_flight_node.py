#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from weather import Weather

def pre_flight():
    pub = rospy.Publisher('weather_data', String, queue_size=10)
    rospy.init_node('pre_flight', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    weather = Weather();
    weather.setLocation(55.471089000000006, 10.330159499999999, 1)

    while not rospy.is_shutdown():
        print(weather.getTemperature() + "C")
        print(weather.getWindDirection())
        print(weather.getWindSpeed() + "m/s")
        print("Wind Category: " + weather.getWindCategory())
        print("Humidity: " + weather.getHumidity() + "%")

        example_str = weather.getWindSpeed() + "m/s"
        pub.publish(example_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        pre_flight()
    except rospy.ROSInterruptException:
        pass
