<?php
    // Require DB config
    require('config.php');
    // Connect to database
    $con = mysqli_connect($mysql_host, $mysql_user, $mysql_pw, $mysql_db);
    if (!$con) {
        die("Connection failed: " . mysqli_connect_error());
    }

    if (isset($_POST['show_uncompleted']) && $_POST['show_uncompleted'] == 1) {
        $sql = "SELECT * FROM `AED_requests` WHERE  `completed` = 0 ORDER BY `int_id` DESC";
    } else {
        $sql = "SELECT * FROM `AED_requests` ORDER BY `int_id` DESC";
    }
    $result = mysqli_query($con, $sql);          //query
    $combinedArray = array();

    if (mysqli_num_rows($result) > 0) {
        while ($row = mysqli_fetch_assoc($result)) {
            $combinedArray[] = $row;
        }
        echo json_encode($combinedArray); // Add "JSON_FORCE_OBJECT" if there shuld be identifiers but note these are "0" ... "max_num"
    } else {
        echo json_encode(0);
    }

    mysqli_close($con);
?>
