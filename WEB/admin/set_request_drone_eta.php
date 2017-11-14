<?php
    // Require DB config
    require('config.php');
    // Connect to database
    $con = mysqli_connect($mysql_host, $mysql_user, $mysql_pw, $mysql_db);
    if (!$con) {
        die("Connection failed: " . mysqli_connect_error());
    }

    $drone_velocity = 6; //unit: m/s
    $take_off_time = 5; //unit: s; taken from second demo video
    $landing_time = 10; //unit: s; taken from second demo video

    if (isset($_POST['request_id'])) {
        $request_id = $_POST['request_id'];
        $drone_id = $_POST['drone_id'];
        $eta = $_POST['eta'];

        if (isset($_POST['drone_id']) && isset($_POST['eta'])) {
            // Update drone_id and eta, based on input
            //echo $_POST['eta'].'-';
            if ($eta == 0) {
                // get the lat and longs fro both inital and target points
                $sql = "SELECT * FROM `AED_drone_list`  WHERE `id` = '$drone_id'";
                $result = mysqli_query($con, $sql);          //query

                if (mysqli_num_rows($result) > 0) {
                    $row = mysqli_fetch_array($result);

                    $lat1 = $row['cur_lat'];
                    $lng1 = $row['cur_lng'];

                    $lat2 = $row['target_lat'];
                    $lng2 = $row['target_lng'];

                    $lat1Rad = deg2rad($lat1);
                    $lat2Rad = deg2rad($lat2);
                    $deltaPhi = deg2rad($lat2-$lat1);
                    $deltaLambda = deg2rad($lng2-$lng1);
                    $a = sin($deltaPhi/2)*sin($deltaPhi/2) + cos($lat1Rad)*cos($lat2Rad)*sin($deltaLambda/2)*sin($deltaLambda/2);
                    $c = 2 * atan2(sqrt($a),sqrt(1-$a));
                    $earthRadius = 6371e3;
                    $distance = $earthRadius*$c;

                    $flying_time = round($distance/$drone_velocity,0); // Calc time, no decimals
                    $final_time = $flying_time+$take_off_time+$landing_time; // Add take-off and landing time

                    $time_str = date("YmdHis"); // Now time in str format
                    $eta = date('YmdHis', strtotime("+".$final_time." seconds", strtotime($time_str))); // Add the time to now time
                    //echo $eta.'-';
                }
            }
            $sql = "UPDATE `AED_requests` SET `drone_id` = $drone_id, `eta` = $eta WHERE  `request_id` = '$request_id'";
        } elseif (isset($_POST['drone_id']) && !isset($_POST['eta'])) {
            // Update drone_id
            $sql = "UPDATE `AED_requests` SET `drone_id` = $drone_id WHERE  `request_id` = '$request_id'";
        } elseif (!isset($_POST['drone_id']) && isset($_POST['eta'])) {
            // Update eta
            $sql = "UPDATE `AED_requests` SET `eta` = $eta WHERE  `request_id` = '$request_id'";
        }

        if (isset($_POST['drone_id']) || isset($_POST['eta'])) {
            $result = mysqli_query($con, $sql);          //query
        }

        if(mysqli_affected_rows($con) == 1) {
            //die('Could not update data: ' . mysql_error());
            $array_out = array(
                "status" => 1,
            );
        } else {
            $array_out = array(
                "status" => 0,
            );
        }
        echo json_encode($array_out);

    } else {
        echo json_encode(0);
    }

   mysqli_close($con);
?>
