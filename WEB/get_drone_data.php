<?php
    // Require DB config
    require('config.php');
    // Connect to database
    $con = mysqli_connect($mysql_host, $mysql_user, $mysql_pw, $mysql_db);
    if (!$con) {
        die("Connection failed: " . mysqli_connect_error());
    }

    if (empty($_POST['drone_id'])) {
        // Return all the drones
        $sql = "SELECT * FROM `AED_drone_list`";
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
        //vardump($result)
    } else {
        // Return the drone identified by the post request 'drone_id'
        $drone_id = $_POST['drone_id'];
        $sql = "SELECT * FROM `AED_drone_list` WHERE `id` = '$drone_id' LIMIT 1";
        $result = mysqli_query($con, $sql);          //query

        $array = mysqli_fetch_row($result);                          //fetch result
        if (empty($array)) {
            echo json_encode(0);
        } else {
            echo json_encode($array);
        }
    }

    mysqli_close($con);
?>
