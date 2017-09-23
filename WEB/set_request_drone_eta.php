<?php
    // Require DB config
    require('config.php');
    // Connect to database
    $con = mysqli_connect($mysql_host, $mysql_user, $mysql_pw, $mysql_db);
    if (!$con) {
        die("Connection failed: " . mysqli_connect_error());
    }

    $request_id = $_POST['request_id'];
    $drone_id = $_POST['drone_id'];
    $eta = $_POST['eta'];


    if (isset($_POST['drone_id']) && isset($_POST['eta'])) {
        $sql = "UPDATE `AED_requests` SET `drone_id` = $drone_id, `eta` = $eta WHERE  `request_id` = '$request_id'";
    } elseif (isset($_POST['drone_id']) && !isset($_POST['eta'])) {
        $sql = "UPDATE `AED_requests` SET `drone_id` = $drone_id WHERE  `request_id` = '$request_id'";
    } elseif (!isset($_POST['drone_id']) && isset($_POST['eta'])) {
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

   mysqli_close($con);
?>
