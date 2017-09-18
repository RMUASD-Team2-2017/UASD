<?php
    if (!empty($_POST['lat']) && !empty($_POST['lat'])) { //check if post using isset
        //--------------------------------------------------------------------------
        // Example php script for fetching data from mysql database
        //--------------------------------------------------------------------------
        require('config.php');
        $tableName = "AED_smartphone_requests";

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
        $lat = $_POST['lat'];
        $lng = $_POST['lng'];
        $GPS_timestamp = $_POST['GPS_timestamp'];

        $loc_accuracy = $_POST['loc_accuracy'];
        $altitude = $_POST['altitude'];
        $altitude_accuracy = $_POST['altitude_accuracy'];

        $ip = $_SERVER['REMOTE_ADDR'];

        $browser = $_SERVER['HTTP_USER_AGENT'];

        $request_id = $_POST['first_GPS_timestamp'];
        $request_id_md5 = md5($request_id.$ip.$browser);
        //echo $request_id_md5;

        $sql = "INSERT INTO `AED_smartphone_requests` (`int_id`, `id`, `loc_lat`, `loc_lon`, `req_time`, `timestamp`, `loc_accuracy`, `altitude`, `altitude_accuracy`) VALUES (NULL, '$request_id_md5', '$lat', '$lng', '$time', '$GPS_timestamp', '$loc_accuracy', '$altitude', '$altitude_accuracy');";
        $result = mysqli_query($con, $sql);          //query

        //echo $_POST['lat'];

        // Get ETA for drone
        $sql = "SELECT * FROM `AED_drone_list` WHERE `target_id` = '$request_id_md5'";
        $result = mysqli_query($con, $sql);          //query
        $array = mysqli_fetch_row($result);                          //fetch result
        //echo ",".$array[12]; //field 13 (count from 1) in AED_drone_list is the ETA
        if (!empty($array)) {
            echo date('Y-m-d G:i:s', strtotime($array[12]));
        } else {
            echo "estimating ...";
        }


        //--------------------------------------------------------------------------
        // 3) close connection
        //--------------------------------------------------------------------------
        mysqli_close($con);
    }
?>
