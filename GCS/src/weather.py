
import urllib2
import xmltodict
from datetime import datetime

class Weather:
    def __init__(self):
        self.longitude = 0
        self.latitude = 0

    def setLocation(self, latitude, longitude, msl):
        self.longitude = str(longitude)
        self.latitude = str(latitude)
        self.msl = str(msl)
        self.mainurl = "http://api.met.no/weatherapi/locationforecastlts/1.3/?lat="

        # Create URL and fetch XML file
        self.fullurl = self.mainurl + self.latitude + ";lon=" + self.longitude + ";msl=" + self.msl
        file = urllib2.urlopen(self.fullurl)
        self.data = file.read()
        file.close()

        # Convert XML to dictionary
        self.data = xmltodict.parse(self.data)

        # Get the weather data fitting the current date/time
        self.getWeatherNow()

    def getTemperature(self):
        return self.data["location"]["temperature"]["@value"]   # [degrees celcius]

    def getWindDirection(self):
        direction = self.data["location"]["windDirection"]["@name"]
        degress = self.data["location"]["windDirection"]["@deg"]
        return degress+direction

    def getWindSpeed(self):
        return self.data["location"]["windSpeed"]["@mps"]   # [m/s]

    def getWindCategory(self):
        categories = ["Calm", "Light Air", "Light Breeze", "Gentle Breeze", "Moderate Breeze", "Fresh Breeze", "Strong Breeze", "High wind", "Gale, fresh gale", "Strong/severe gale", "Storm", "Violent storm", "Hurricane"]
        windCat = int(self.data["location"]["windSpeed"]["@beaufort"])
        return categories[windCat]

    def getHumidity(self):
        return self.data["location"]["humidity"]["@value"]

    def printData(self):
        print self.data

    def printUrl(self):
        print self.fullurl

    def findNearestDate(self, dates, date):
        return min(dates, key=lambda x: abs(x - date))

    def getWeatherNow(self):
        self.data = self.data["weatherdata"]["product"]["time"] # Only keep the necessary data
        dates = []
        newData = []

        # Extract all data with the same from and to dates/times
        for x in self.data:
            if(x["@from"] == x["@to"]):
                newData.append(x)
                date = datetime.strptime(x["@from"], "%Y-%m-%dT%H:%M:%SZ")   # Convert date and time to a datetime object
                dates.append(date)

        # Find the date/time from the dataset which is nearest to the current date/time
        datenow = datetime.now()
        nearestDate = self.findNearestDate(dates, datenow)

        # Extract all data from this new found date/time
        for x in newData:
            date = datetime.strptime(x["@from"], "%Y-%m-%dT%H:%M:%SZ")   # Convert date and time to a datetime object
            if(date == nearestDate):
                self.data = x
