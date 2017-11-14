<?php
    // Require DB config
    require('config.php');
    // Connect to database
    $con = mysqli_connect($mysql_host, $mysql_user, $mysql_pw, $mysql_db);
    if (!$con) {
        die("Connection failed: " . mysqli_connect_error());
    }

    $drone_id = $_POST['drone_id'];

    $cur_lat = $_POST['cur_lat'];
    $cur_lng = $_POST['cur_lng'];
    $target_lat = $_POST['target_lat'];
    $target_lng = $_POST['target_lng'];
    $path = $_POST['path'];
    $weather = $_POST['weather'];

    $sql = "UPDATE `AED_drone_list` SET `loc_lat` = '$cur_lat', `loc_lng` = '$cur_lng', `cur_lat` = '$cur_lat', `cur_lng` = '$cur_lng', `target_lat` = '$target_lat', `target_lng` = '$target_lng', `path` = '$path', `weather` = '$weather' WHERE  `id` = '$drone_id'";
    $result = mysqli_query($con, $sql);          //query

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
