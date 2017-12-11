
// Global variables
var map; // map object, not yet initialized

var marker_human = false; // marker object, not yet initialized, therefore false since the code can react on this when the user has not marked a location
var marker_human_help_icon = 'https://www.techgen.dk/AED/marker_human_help.png';

var LatLng_HCAAirport;
var LatLng_OdenseModelfly;
var LatLng_HCAGolf;

var LatHolder = document.getElementById("lat");
var LngHolder = document.getElementById("lng");
var statusHolder = document.getElementById('sendStatus');

// Map initializer, callback from the Google Map API
function myMap() {
    // Save positions since the google object is first to be used in the callback otherwise it is not known
    LatLng_HCAAirport = new google.maps.LatLng(55.471910, 10.325374);
    LatLng_OdenseModelfly = new google.maps.LatLng(55.472059, 10.414367);
    LatLng_HCAGolf = new google.maps.LatLng(55.558709, 10.108918);

    var mapProp= {
        //center:new google.maps.LatLng(51.508742,-0.120850),
        center: LatLng_HCAAirport,
        mapTypeId: 'hybrid',
        zoom: 16,
        scaleControl: false,
        streetViewControl: false,
        mapTypeControl: false,
    };
    map = new google.maps.Map(document.getElementById("googleMap"),mapProp);

    // Listen for any clicks on the map.
    // Source: http://thisinterestsme.com/google-maps-api-location-picker-example/
    google.maps.event.addListener(map, 'click', function(event) {
        document.getElementById("sendReq").disabled = false;
        //Get the location that the user clicked.
        var clickedLocation = event.latLng;
        //If the marker hasn't been added.
        if(marker_human === false){

            marker_human_help_icon_google = {
                url: marker_human_help_icon,
                // This marker is 30 pixels wide by 36 pixels high.
                size: new google.maps.Size(30, 36),
                // The origin for this image is (0, 0).
                origin: new google.maps.Point(0, 0),
                // The anchor for this image is the center which is at (15, 17).
                anchor: new google.maps.Point(15, 17)
            };
            marker_human = new google.maps.Marker({
                position: clickedLocation,
                map: map,
                draggable: true, //make it draggable
                icon: marker_human_help_icon_google
            });

            //Listen for drag events!
            google.maps.event.addListener(marker_human, 'dragend', function(event){
                markerLocation();
            });
        } else{
            //Marker has already been added, so just change its location.
            marker_human.setPosition(clickedLocation);
        }
        //Get the marker's location.
        markerLocation();
    });
}

// This function will get the marker's current location and then add the lat/long values to our textfields so that we can save the location. It also pans to the location
// Source: http://thisinterestsme.com/google-maps-api-location-picker-example/
function markerLocation(){
    //Get location.
    var currentLocation = marker_human.getPosition();
    //Add lat and lng values to a field that we can save.
    document.getElementById('lat').value = currentLocation.lat().toFixed(5); //latitude
    document.getElementById('lng').value = currentLocation.lng().toFixed(5); //longitude
    //map.panTo(currentLocation); // Set map new center by panning to it
}

function changeLocation(id) {
    if (id == 1) {
        map.panTo(LatLng_HCAAirport);
    } else if (id == 2) {
        map.panTo(LatLng_OdenseModelfly);
    } else if (id == 3) {
        map.panTo(LatLng_HCAGolf);
    }
}

function sendRequest() {
    //document.getElementById("sendReq").disabled = true;

    console.log("lat:"+document.getElementById('lat').value);
    console.log("lng:"+document.getElementById('lng').value);
    console.log("loc precision:"+document.getElementById('loc_precision').value);
    console.log("alt:"+document.getElementById('alt').value);
    console.log("alt precision:"+document.getElementById('alt_precision').value);
    console.log(Date.now());

    statusHolder.innerHTML = "<i>sending</i>";

    $.ajax({
        url: "../add_smartphone_request.php",
        type: "POST",
        dataType: 'json', // or 'text'
        //contentType: 'application/x-www-form-urlencoded',
        data: { lat: document.getElementById('lat').value,
            lng: document.getElementById('lng').value,
            GPS_timestamp: Date.now(),
            loc_accuracy: document.getElementById('loc_precision').value,
            altitude: document.getElementById('alt').value,
            altitude_accuracy: document.getElementById('alt_precision').value,
            first_GPS_timestamp: Date.now()},
        success: function(msg) {
            console.log("Response received!");
            statusHolder.innerHTML = "<i>succes</i>";
            setTimeout(function(){ clearStatusHolder(); }, 1000); // run the getLocaiton method again after 1s
            var rxArray = JSON.parse(JSON.stringify(msg));
        },
        error: function(jqXHR, textStatus, errorThrown) {
           console.log(textStatus, errorThrown);
        }
    });
}

function clearStatusHolder() {
    statusHolder.innerHTML = "";
}
