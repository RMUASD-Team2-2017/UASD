<?php
    if (isset($_POST['drone_id']) && $_POST['drone_id'] != '') {
        // Require DB config
        require('config.php');
        // Connect to database
        $con = mysqli_connect($mysql_host, $mysql_user, $mysql_pw, $mysql_db);
        if (!$con) {
            die("Connection failed: " . mysqli_connect_error());
        }

        $drone_id = $_POST['drone_id'];

        $sql = "SELECT * FROM `AED_requests`  WHERE `drone_id` = '$drone_id' AND `approved` !=0 AND `approved` != 2 AND `completed` = 0 ORDER BY `approved` DESC";

        //$sql = "SELECT * FROM `AED_requests`  WHERE `drone_id` = '$drone_id' AND `approved` !=0 AND `approved` != 2 ORDER BY `int_id` DESC";

        $result = mysqli_query($con, $sql);          //query
        $combinedArray = array();

        if (mysqli_num_rows($result) > 0) {
            // This is sort of a quick fix since we take the request which has been approved last and then we do not look at the rest'!
            $row = mysqli_fetch_assoc($result);
            echo json_encode($row['stopped']); // Add "JSON_FORCE_OBJECT" if there shuld be identifiers but note these are "0" ... "max_num"
        } else {
            echo 'test';
            echo json_encode(0);
        }

        mysqli_close($con);
    } else {
        echo 'test2';
        echo json_encode(0);
    }
?>
