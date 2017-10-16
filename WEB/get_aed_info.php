<?php
    if (!empty($_POST['first_GPS_timestamp'])) { //check if post using isset
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
        $ip = $_SERVER['REMOTE_ADDR'];

        $browser = $_SERVER['HTTP_USER_AGENT'];

        $request_id = $_POST['first_GPS_timestamp'];
        $request_id_md5 = md5($request_id.$ip.$browser);
        //echo $request_id_md5;

        // Set default
        $return_time = "estimating";
        $drone_state = "dispatching";

        $drone_lat = 0;
        $drone_lng = 0;

        // See if a drone has been dispatched
        $sql = "SELECT * FROM `AED_requests` WHERE `request_id` = '$request_id_md5'";
        $result = mysqli_query($con, $sql);          //query
        $array = mysqli_fetch_row($result);                          //fetch result
        if (!empty($array[1])) {
            // Drone has been dispatched
            $return_time = date('Y-m-d H:i:s', strtotime($array[4]));

            // Get drone location
            $sql = "SELECT * FROM `AED_drone_list` WHERE `id` = '$array[1]'";
            $result = mysqli_query($con, $sql);          //query
            $array = mysqli_fetch_row($result);          //fetch result
            $drone_lat = $array[7];
            $drone_lng = $array[8];


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
            "lat" => $drone_lat,
            "lng" => $drone_lng,
        );
        echo json_encode($array_out);


        //--------------------------------------------------------------------------
        // 3) close connection
        //--------------------------------------------------------------------------
        mysqli_close($con);
    }
?>
