<?php
    // Require DB config
    require('config.php');
    // Connect to database
    $con = mysqli_connect($mysql_host, $mysql_user, $mysql_pw, $mysql_db);
    if (!$con) {
        die("Connection failed: " . mysqli_connect_error());
    }

    $request_id = $_GET['id'];
    if (isset($_GET['id'])) {
        $sql = "UPDATE `AED_requests` SET `approved` = '$time'  WHERE `request_id` = '$request_id'";
        $result = mysqli_query($con, $sql);          //query
    }

    mysqli_close($con);

    header("Location: show_requests.php");
    die();
?>
