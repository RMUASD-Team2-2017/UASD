<?php
    // Require DB config
    require('config.php');
    // Connect to database
    $con = mysqli_connect($mysql_host, $mysql_user, $mysql_pw, $mysql_db);
    if (!$con) {
        die("Connection failed: " . mysqli_connect_error());
    }

    $request_id = $_GET['id'];
    $severity = $_GET['severity'];
    
    if (isset($_GET['id'])) {
        $sql = "UPDATE `AED_requests` SET `stopped` = '$severity'  WHERE `request_id` = '$request_id'";
        $result = mysqli_query($con, $sql);          //query
    }

    mysqli_close($con);

    $page = $_GET['page'];
    if ($page == 'show_request.php') {
        header("Location: ".$page."?id=".$request_id);
    } else {
        header("Location: ".$page);
    }

    die();
?>
