<?php
    if (!empty($_POST['lat']) && !empty($_POST['lat'])) { //check if post using isset
        // Require DB config
        require('config.php');

        //--------------------------------------------------------------------------
        // 1) Connect to mysql database
        //--------------------------------------------------------------------------
        $con = mysqli_connect($mysql_host, $mysql_user, $mysql_pw, $mysql_db);
        if (!$con) {
            die("Connection failed: " . mysqli_connect_error());
        }

        //--------------------------------------------------------------------------
        // 2) Query database for data
        //--------------------------------------------------------------------------
        $lat = round((float)$_POST['lat'],5);
        $lng = round((float)$_POST['lng'],5);
        $GPS_timestamp = $_POST['GPS_timestamp'];

        $loc_accuracy = $_POST['loc_accuracy'];
        $altitude = $_POST['altitude'];
        $altitude_accuracy = $_POST['altitude_accuracy'];

        $ip = $_SERVER['REMOTE_ADDR'];

        $browser = $_SERVER['HTTP_USER_AGENT'];

        $request_id = $_POST['first_GPS_timestamp'];
        $request_id_md5 = md5($request_id.$ip.$browser);
        //echo $request_id_md5;

        // Set default
        $return_time = "estimating";
        $drone_state = "dispatching";

        if ($GPS_timestamp == $request_id) {
            // Check if it is completely new information / a new request
            $sql = "INSERT INTO `AED_requests` (`int_id`, `drone_id`, `request_id`, `time`, `completed`, `eta`, `approved`, `stopped`) VALUES (NULL, '', '$request_id_md5', '$time', '0', '', '0', '0');";
            $result = mysqli_query($con, $sql);          //query
        } else {
            // See if a drone has been dispatched
            $sql = "SELECT * FROM `AED_requests` WHERE `request_id` = '$request_id_md5'";
            $result = mysqli_query($con, $sql);          //query
            $array = mysqli_fetch_row($result);                          //fetch result
            if (!empty($array[1])) {
                // Drone has been dispatched
                $return_time = date('Y-m-d H:i:s', strtotime($array[4]));
                // Get drone state
                    //$sql = "SELECT * FROM `AED_drone_list` WHERE `id` = '$array[1]'";
                    //$result = mysqli_query($con, $sql);          //query
                    //$array = mysqli_fetch_row($result);          //fetch result
                    //$drone_state = $array[6]; // Drone state
                if ($array[6] == 'landed') {
                    $drone_state = "landed";
                } else {
                    $drone_state = "dispatched";
                }
            }
        }


        // // Check if it is completely new information / a new request
        // $sql = "SELECT * FROM `AED_requests` WHERE `request_id` = '$request_id_md5'";
        // //$sql = "SELECT EXISTS(SELECT 1 FROM `AED_requests` WHERE `request_id` = '$request_id_md5');";
        // $result = mysqli_query($con, $sql);          //query
        // //$array = mysqli_fetch_row($result);                          //fetch result
        // //if (empty($array)) {
        // if (mysqli_num_rows($result) == 0) {
        //     $sql = "INSERT INTO `AED_requests` (`int_id`, `drone_id`, `request_id`, `time`, `completed`, `eta`) VALUES (NULL, '', '$request_id_md5', '$time', '0', '');";
        //     $result = mysqli_query($con, $sql);          //query
        // } else {
        //     // See if a drone has been dispatched
        //     $array = mysqli_fetch_row($result);                          //fetch result
        //     if (!empty($array[1])) {
        //         // Drone has been dispatched
        //         $return_time = date('Y-m-d H:i:s', strtotime($array[4]));
        //         // Get drone state
        //             //$sql = "SELECT * FROM `AED_drone_list` WHERE `id` = '$array[1]'";
        //             //$result = mysqli_query($con, $sql);          //query
        //             //$array = mysqli_fetch_row($result);          //fetch result
        //             //$drone_state = $array[6]; // Drone state
        //         $drone_state = "dispatched";
        //     }
        // }

        // Make a new entry for the new information
        $sql = "INSERT INTO `AED_smartphone_requests` (`int_id`, `id`, `loc_lat`, `loc_lng`, `req_time`, `timestamp`, `loc_accuracy`, `altitude`, `altitude_accuracy`) VALUES (NULL, '$request_id_md5', '$lat', '$lng', '$time', '$GPS_timestamp', '$loc_accuracy', '$altitude', '$altitude_accuracy');";
        $result = mysqli_query($con, $sql);          //query

        // See if a drone has been dispatched
        // $sql = "SELECT * FROM `AED_requests` WHERE `request_id` = '$request_id_md5'";
        // $result = mysqli_query($con, $sql);          //query
        // $array = mysqli_fetch_row($result);                          //fetch result


        // //echo ",".$array[12]; //field 13 (count from 1) in AED_drone_list is the ETA
        // if (!empty($array)) {
        //     $return_time = date('Y-m-d G:i:s', strtotime($array[12]));
        //     $drone_state = $array[6];
        // } else {
        //     $return_time = "estimating";
        //     $drone_state = "dispatching";
        // }
        $array_out = array(
            "ETA" => $return_time,
            "AED_status" => $drone_state,
        );
        echo json_encode($array_out);


        //--------------------------------------------------------------------------
        // 3) close connection
        //--------------------------------------------------------------------------
        mysqli_close($con);
    }
?>
