<!DOCTYPE html>
<html>
    <head>
        <!-- <style type="text/css">
        #googleMap{ width:700px; height: 500px; }
        </style> -->
        <link rel="stylesheet" href="style.css">
        <title>UASD Location Pick Test</title>
    </head>
    <body>
        <div class="centerDiv">
            <center><a href="../">Request AED</a> - <a href="show_drones.php">Drones</a> - <a href="show_requests.php">Requests</a> - <b><i>Request AED (backend)</i></b> - <a href="db.log"><i>Database exception log</i></a></center>
            <center><h1>Request AED (backend)</h1></center>

            <h2>Choose location:</h2>
            <button onclick="changeLocation(1)">HCA Airport</button>
            <button onclick="changeLocation(2)">Odense Modelfly</button>
            <button onclick="changeLocation(3)">HCA Golf</button>
            <br>

            <h2>Request parameters</h2>
            <p>
                Latitude: <input type="text" id="lat" readonly="yes"><br>
                Longitude: <input type="text" id="lng" readonly="yes"><br>
                Location accuracy: <input type="text" id="loc_precision" value="5">m<br>
                Altitude: <input type="text" id="alt" value="0">m<br>
                Altitude accuracy: <input type="text" id="alt_precision" value="0">m
            </p>
            <button onclick="sendRequest()" id="sendReq" disabled>SEND REQUEST</button>
            <p id="sendStatus" style="color:gray;"></p>

            <br>
            <br>

            <div id="googleMap" style="width:100%;height:400px;"></div>
        </div>




        <script src="https://maps.googleapis.com/maps/api/js?key=AIzaSyDTHa8VL03UUNRQdlxNm99TaUzTsK2v6Ls&callback=myMap"
            async defer></script>
        <!-- <script type="text/javascript" src="https://maps.googleapis.com/maps/api/js"></script> -->
        <script src="//ajax.googleapis.com/ajax/libs/jquery/1.8.1/jquery.min.js"></script>
        <script type="text/javascript" src="map-and-sender.js"></script>
    </body>
</html>
