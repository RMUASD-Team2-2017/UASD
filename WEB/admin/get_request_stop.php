<?php
    if (isset($_POST['request_id']) && $_POST['request_id'] != '') {
        // Require DB config
        require('config.php');
        // Connect to database
        $con = mysqli_connect($mysql_host, $mysql_user, $mysql_pw, $mysql_db);
        if (!$con) {
            die("Connection failed: " . mysqli_connect_error());
        }

        $request_id = $_POST['request_id'];

        $sql = "SELECT * FROM `AED_requests`  WHERE `request_id` = '$request_id' ORDER BY `int_id` ASC";

        $result = mysqli_query($con, $sql);          //query
        $combinedArray = array();

        if (mysqli_num_rows($result) > 0) {
            $row = mysqli_fetch_assoc($result);
            echo json_encode($row['stopped']); // Add "JSON_FORCE_OBJECT" if there shuld be identifiers but note these are "0" ... "max_num"
        } else {
            echo json_encode(0);
        }

        mysqli_close($con);
    } else {
        echo json_encode(0);
    }
?>
