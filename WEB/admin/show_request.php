<?php
    if ($_GET['id'] != '') {
        // Require DB config
        require('config.php');
        // Connect to database
        $con = mysqli_connect($mysql_host, $mysql_user, $mysql_pw, $mysql_db);
        if (!$con) {
            die("Connection failed: " . mysqli_connect_error());
        }

        $page = $_SERVER['PHP_SELF'];
        $sec = "5";

        $request_id = $_GET['id'];

        $sql = "SELECT * FROM `AED_requests` WHERE  `request_id` = '$request_id' ORDER BY  `int_id` DESC";
        $result = mysqli_query($con, $sql);          //query

        $row = mysqli_fetch_array($result);
        $request_time = date('Y-m-d H:i:s', strtotime($row['time']));
        $request_eta = $row['eta'];
        $request_approved = $row['approved'];
        $request_completed = $row['completed'];
        $request_drone = $row['drone_id'];
        $request_internal_id = $row['int_id'];


        echo '<!DOCTYPE html>
        <html>
            <head>
                <link rel="stylesheet" href="style.css">
                <link rel="stylesheet" href="table.css">
                <meta http-equiv="refresh" content="'.$sec.';URL='.$page.'?id='.$request_id.'">
            </head>
        <body>
        <div class="centerDiv">
        <center><a href="show_requests.php">⬑ Go Back</a></center>
        <center><h1>AED Request '.$request_internal_id.'</h1></center>';

        echo '<h2>Administrative information</h2>';
        echo '<p><span style="padding:5px; border: 1px solid #CCCCCC; background-color: #AAAAAA;">Unique ID: <b>'.$request_id.'</b></span></p>';
        echo '<p><span style="padding:5px; border: 1px solid #CCCCCC; background-color: #AAAAAA;">Time and date: <b>'.$request_time.'</b></span></p>';

        if ($request_approved == 0) {
            echo '<p><span style="padding:5px; border: 1px solid #CCCCCC; background-color: #FF0000;">Request awaiting approval</span></p>';
            echo '<p style="margin-bottom: 20px;">';
            echo '<a href="https://www.techgen.dk/AED/admin/approve_request.php?id='. $request_id . '&page=show_request.php' .'" class="button_style"><span style="padding:5px; margin-left: 25px; border: 1px dashed #CCCCCC; background-color: #c0edc0; border-radius: 6px;">APPROVE</span></a>';
            echo '<a href="https://www.techgen.dk/AED/admin/reject_request.php?id='. $request_id . '&page=show_request.php' . '" class="button_style"><span style="padding:5px; margin-left: 5px; border: 1px dashed #CCCCCC; background-color: #ff7676; border-radius: 6px;">REJECT</span></a>';
            echo '</p>';
        } elseif ($request_approved == 2) {
            echo '<p><span style="padding:5px; border: 1px solid #CCCCCC; background-color: #FFFF00;">Request rejected</span></p>';
        } else {
            echo '<p><span style="padding:5px; border: 1px solid #CCCCCC; background-color: #90EE90;">Request approved at <b>'.date('Y-m-d H:i:s', strtotime($request_approved)).'</b></span></p>';
        }

        if ($request_completed == 0) {
            echo '<p><span style="padding:5px; border: 1px solid #CCCCCC; background-color: #FF0000;">Request awaiting completion</span></p>';
            if ($request_approved != 0 && $request_approved != 2) {
                echo '<p style="margin-bottom: 20px;">';
                echo '<a href="https://www.techgen.dk/AED/admin/complete_request.php?id='. $request_id . '&page=show_request.php' .'" class="button_style"><span style="padding:5px; margin-left: 25px; border: 1px dashed #CCCCCC; background-color: #c0edc0; border-radius: 6px;">COMPLETE</span></a>';
                echo '</p>';
            }
        } else {
            echo '<p><span style="padding:5px; border: 1px solid #CCCCCC; background-color: #90EE90;">Request completed at <b>'.date('Y-m-d H:i:s', strtotime($request_completed)).'</b></span></p>';
        }


        $sql = "SELECT * FROM `AED_drone_list`  WHERE `id` = '$request_drone'";
        $result = mysqli_query($con, $sql);          //query
        $row = mysqli_fetch_array($result);
        $drone_lat = $row['cur_lat'];
        $drone_lng = $row['cur_lng'];
        $gcs_lat = $row['loc_lat'];
        $gcs_lng = $row['loc_lng'];

        if ($request_drone == 0) {
            echo '<p><span style="padding:5px; border: 1px solid #CCCCCC; background-color: #FF0000;">No UAV has been assigned</span></p>';
        } else {
            echo '<p><span style="padding:5px; border: 1px solid #CCCCCC; background-color: #90EE90;">Assigned UAV: <b>'.$row['name'].'</b> - <i>UAV ID: '.$request_drone.'</i></span></p>';

            if ($row['state'] == 'idle') {
                echo '<p><span style="padding:5px; border: 1px solid #CCCCCC; background-color: #AAAAAA;" >UAV state: <b>' . $row['state'] . "</b></span></p>";
            } elseif ($row['state'] == 'maintenance' || $row['state'] == 'error') {
                echo '<p><span style="padding:5px; border: 1px solid #CCCCCC; background-color: #FF0000;" >UAV state: <b>' . $row['state'] . "</b></span></p>";
            } elseif ($row['state'] == 'landed') {
                echo '<p><span style="padding:5px; border: 1px solid #CCCCCC; background-color: #90EE90;" >UAV state: <b>' . $row['state'] . "</b></span></p>";
            } else {
                echo '<p><span style="padding:5px; border: 1px solid #CCCCCC; background-color: #FFFF00;" >UAV state: <b>' . $row['state'] . "</b></span></p>";
            }
            if ($row['state'] != 'landed' && $row['state'] != 'maintenance' && $row['state'] != 'error') {
                if ($request_eta != '0') {
                    echo '<p><span style="padding:5px; border: 1px solid #CCCCCC; background-color: #90EE90;">ETA: <b>'.date('Y-m-d H:i:s', strtotime($request_eta)).'</b></span></p>';
                } else {
                    echo '<p><span style="padding:5px; border: 1px solid #CCCCCC; background-color: #FFFF00;">ETA: none</span></p>';
                }
            }

        }

        $sql = "SELECT * FROM `AED_smartphone_requests`  WHERE `id` = '$request_id'";
        $result = mysqli_query($con, $sql);          //query

        if (mysqli_num_rows($result) != 0) {
            $accuracy;
            $best_location_id = 0;
            $first_row = 1;
            $best_row;
            $cntr = 0;

            while($row = mysqli_fetch_array($result))
            {
                if ($first_row == 1) {
                        $accuracy = $row['loc_accuracy'];
                        $first_row = 0;
                        $best_row = $row;
                } else {
                    if ($row['loc_accuracy'] <= $accuracy) {
                        $accuracy = $row['loc_accuracy'];
                        $best_location_id = $cntr;
                        $best_row = $row;
                    }
                }
                $cntr = $cntr + 1;
            }
        }
        mysqli_data_seek( $result, 0 ); //reset mysqli result to be used again


        echo '<h2>Location information</h2>';
        if (mysqli_num_rows($result) != 0) {
            echo '<table class="customTableClass">
            <thead>
            <tr>
            <th>Request #</th>
            <th>Location</th>
            <th>Location accuracy</th>
            <th>Time and date</th>
            <th>Altitude</th>
            <th>Altitude accuracy</th>
            </tr>
            </thead>
            <tbody>';

            $local_itr = 1;
            while($row = mysqli_fetch_array($result))
            {
                if ($local_itr == $best_location_id+1) {
                    echo '<tr style="background: #A9FFA6;">';
                } else {
                    echo '<tr>';
                }
                echo "<td>" . $local_itr . "</td>";
                echo "<td>" . $row['loc_lat'] . ", " . $row['loc_lng'] . "</td>";
                echo "<td>" . $row['loc_accuracy'] . "</td>";
                echo "<td>" . date('Y-m-d ', strtotime($row['req_time'])) . "<b>" . date('H:i:s', strtotime($row['req_time'])) . "</b>" . "</td>";
                echo "<td>" . $row['altitude'] . "</td>";
                echo "<td>" . $row['altitude_accuracy'] . "</td>";
                echo "</tr>";
                $local_itr = $local_itr + 1;
            }
            echo "</tbody>
            </table>";
        } else {
            echo '<p style="color:red;">There is no entries concerning the specified request id.</p>';
        }

        echo '<div id="googleMap" style="width:100%;height:400px;margin-top:15px;"></div>';
        echo '<script src="//ajax.googleapis.com/ajax/libs/jquery/1.8.1/jquery.min.js"></script>';

        if ($request_approved != 0 && $request_completed == 0 && $request_drone != 0) {
            echo '<script>
                var marker_human_help_icon = \'https://www.techgen.dk/AED/marker_human_help.png\';
                var marker_aed_icon = \'https://www.techgen.dk/AED/marker_aed.png\';
                var marker_gcs_icon = \'https://www.techgen.dk/AED/marker_gcs.png\';

                var marker_human_help_icon_google;
                var marker_aed_icon_google;
                var marker_gcs_icon_google;

                function myMap() {
                    var myLatLng = {lat: '.$best_row['loc_lat'].',lng: '.$best_row['loc_lng'].'};
                    var DroneLatLng = {lat: '.$drone_lat.',lng: '.$drone_lng.'};
                    var GCSLatLng = {lat: '.$gcs_lat.',lng: '.$gcs_lng.'};
                    var AvgLatLng = new google.maps.LatLng('. ($best_row['loc_lat']+$gcs_lat)/2 .', '. ($best_row['loc_lng']+$gcs_lng)/2 .');
                    if (myLatLng != null) {
                        // firstRunMap = 0;
                        // Create a map object and specify the DOM element for display.
                        // Similar way to exclude additional show: var map = new google.maps.Map(document.getElementById(\'map\'), {
                        var mapProp= {
                            //center:new google.maps.LatLng(51.508742,-0.120850),
                            center: AvgLatLng,
                            mapTypeId: \'hybrid\',
                            zoom:16,
                            scaleControl: false,
                            streetViewControl: false,
                            mapTypeControl: true,
                        };
                        map=new google.maps.Map(document.getElementById("googleMap"),mapProp);

                        marker_human_help_icon_google = {
                          url: marker_human_help_icon,
                          size: new google.maps.Size(30, 36),
                          origin: new google.maps.Point(0, 0),
                          anchor: new google.maps.Point(15, 17)
                        };
                        marker_human = new google.maps.Marker({
                          position: myLatLng,
                          map: map,
                          title: \'Your Position\',
                          icon: marker_human_help_icon_google,
                          visible: true
                        });

                        marker_aed_icon_google = {
                          url: marker_aed_icon,
                          size: new google.maps.Size(30, 36),
                          origin: new google.maps.Point(0, 0),
                          anchor: new google.maps.Point(15, 17)
                        };
                        marker_aed = new google.maps.Marker({
                          position: DroneLatLng,
                          map: map,
                          title: \'AED Position\',
                          icon: marker_aed_icon_google
                        });

                        marker_gcs_icon_google = {
                          url: marker_gcs_icon,
                          size: new google.maps.Size(30, 36),
                          origin: new google.maps.Point(0, 0),
                          anchor: new google.maps.Point(15, 17)
                        };
                        marker_gcs = new google.maps.Marker({
                          position: GCSLatLng,
                          map: map,
                          title: \'GCS Position\',
                          icon: marker_gcs_icon
                        });

                    }
                }
                </script>';
        } else {
            echo '<script>
                var marker_human_help_icon = \'https://www.techgen.dk/AED/marker_human_help.png\';

                var marker_human_help_icon_google;

                function myMap() {
                    var myLatLng = {lat: '.$best_row['loc_lat'].',lng: '.$best_row['loc_lng'].'};
                    var myLatLng2 = new google.maps.LatLng('.$best_row['loc_lat'].', '.$best_row['loc_lng'].');
                    if (myLatLng != null) {
                        // firstRunMap = 0;
                        // Create a map object and specify the DOM element for display.
                        // Similar way to exclude additional show: var map = new google.maps.Map(document.getElementById(\'map\'), {
                        var mapProp= {
                            //center:new google.maps.LatLng(51.508742,-0.120850),
                            center: myLatLng,
                            mapTypeId: \'hybrid\',
                            zoom:16,
                            scaleControl: false,
                            streetViewControl: false,
                            mapTypeControl: true,
                        };
                        map=new google.maps.Map(document.getElementById("googleMap"),mapProp);

                        marker_human_help_icon_google = {
                          url: marker_human_help_icon,
                          // This marker is 20 pixels wide by 32 pixels high.
                          size: new google.maps.Size(30, 36),
                          // The origin for this image is (0, 0).
                          origin: new google.maps.Point(0, 0),
                          // The anchor for this image is the base of the flagpole at (0, 32).
                          anchor: new google.maps.Point(15, 17)
                        };
                        marker_human = new google.maps.Marker({
                          position: myLatLng,
                          map: map,
                          title: \'Your Position\',
                          icon: marker_human_help_icon_google,
                          visible: true
                        });
                        marker_human.setMap(map);
                        marker_human.setPosition(myLatLng); // Set marker new location

                    }
                }
                </script>';
        }
        echo '<script src="https://maps.googleapis.com/maps/api/js?key=AIzaSyDTHa8VL03UUNRQdlxNm99TaUzTsK2v6Ls&callback=myMap" async defer></script>';
        echo '<p><i>The map shows the latest and best result; marked green in the table above.</i></p>';

        echo '<br />';
        echo 'Updated: '.date('Y-m-d H:i:s', time()).' (refreshes every '.$sec.' seconds)';
        echo '</div>
        </body>
        </html>';

        mysqli_close($con);
    } else {
        echo '<!DOCTYPE html>
        <html>
        <link rel="stylesheet" href="style.css">
        <link rel="stylesheet" href="table.css">
        <body>
        <div class="centerDiv">
        <center><h1>AED Request</h1><br /><h2 style="color:red;">No request id specified</h2></center>';

        echo '</div>
        </body>
        </html>';
    }

?>
