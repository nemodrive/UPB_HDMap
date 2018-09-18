import utm
import numpy as np
import json
import urllib.request

def elevation(lat, lng):
    apikey = "AIzaSyDAeuq1vXAszCNF9HhLIadyToJ44OSJMMQ"
    url = "https://maps.googleapis.com/maps/api/elevation/json"
    request = urllib.request.urlopen(url+"?locations="+str(lat)+","+str(lng)+"&key="+apikey)
    results = json.load(request).get('results')
    elevation = results[0].get('elevation')
    return elevation

def toLatLon(points):
    pointsLL = []
    for i in range(len(points)):
        pointsLL.append(utm.to_latlon(points[i][0], points[i][1], 35, 'T'))
    pointsLL = np.array(pointsLL)
    return pointsLL
