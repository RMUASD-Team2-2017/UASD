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
        $sec = "10";

        $request_id = $_GET['id'];

        $sql = "SELECT * FROM `AED_requests` WHERE  `request_id` = '$request_id' ORDER BY  `int_id` DESC";
        $result = mysqli_query($con, $sql);          //query

        $row = mysqli_fetch_array($result);
        $request_time = date('Y-m-d H:i:s', strtotime($row['time']));
        $request_eta = $row['eta'];
        $request_approved = $row['approved'];
        $request_completed = $row['completed'];
        $request_drone = $row['drone_id'];


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
        <center><h1>AED Request</h1><h2>Request id: '.$request_id.'</h2></center>';

        echo '<h2>Administrative information</h2>';
        echo '<p>Time and date: '.$request_time.'</p>';

        if ($request_approved == 0) {
            echo '<p>Request awaiting approval (';
            echo '<a href="https://www.techgen.dk/AED/admin/approve_request.php?id='. $request_id . '&page=show_request.php' .'">approve</a>, ';
            echo '<a href="https://www.techgen.dk/AED/admin/reject_request.php?id='. $request_id . '&page=show_request.php' . '">reject</a>)</p>';
        } elseif ($request_approved == 2) {
            echo '<p>Request rejected</p>';
        } else {
            echo '<p>Request approved at '.date('Y-m-d H:i:s', strtotime($request_approved)).'</p>';
        }

        if ($request_completed == 0) {
            echo '<p>Request awaiting completion</p>';
        } else {
            echo '<p>Request completed at '.date('Y-m-d H:i:s', strtotime($request_completed)).'</p>';
        }

        if ($request_drone == 0) {
            echo '<p>No UAV has been assigned</p>';
        } else {
            $sql = "SELECT * FROM `AED_drone_list`  WHERE `id` = '$request_drone'";
            $result = mysqli_query($con, $sql);          //query
            $row = mysqli_fetch_array($result);
            echo '<p>Assigned UAV: '.$row['name'].' - UAV ID: '.$request_drone;
            if ($request_eta != '0') {
                echo '<p>ETA: '.date('Y-m-d H:i:s', strtotime($request_eta)).'</p>';
            } else {
                echo '<p>ETA: none</p>';
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
            <th>GPS timestamp</th>
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
                echo "<td>" . date('Y-m-d H:i:s', strtotime($row['req_time'])) . "</td>";
                echo "<td>" . $row['timestamp'] . "</td>";
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
        echo '<script>
            var marker_human_help = \'https://www.techgen.dk/AED/marker_human_help.png\';
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

                    marker_human = new google.maps.Marker({
                      position: myLatLng,
                      map: map,
                      title: \'Your Position\',
                      icon: marker_human_help,
                      visible: true
                    });
                    marker_human.setMap(map);
                    marker_human.setPosition(myLatLng); // Set marker new location

                }
            }
            </script>';
        echo '<script src="https://maps.googleapis.com/maps/api/js?key=AIzaSyDTHa8VL03UUNRQdlxNm99TaUzTsK2v6Ls&callback=myMap" async defer></script>';
        echo '<p>The map shows the latest and best result; marked green in the table above.</p>';

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
