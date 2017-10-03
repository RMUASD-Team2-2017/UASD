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

        $sql = "SELECT * FROM `AED_smartphone_requests`  WHERE `id` = '$request_id' ORDER BY `int_id` ASC";

        $result = mysqli_query($con, $sql);          //query

        $accuracy;
        $best_location_id = 0;
        $first_row = 1;
        $best_row;

        $cntr = 0;
        if (mysqli_num_rows($result) > 0) {
            while ($row = mysqli_fetch_assoc($result)) {
                if ($first_row == 1) {
                        $accuracy = $row['loc_accuracy'];
                        $first_row = 0;
                } else {
                    if ($row['loc_accuracy'] <= $accuracy) {
                        $accuracy = $row['loc_accuracy'];
                        $best_location_id = $cntr;
                        $best_row = $row;
                    }
                }
                $cntr = $cntr + 1;
            }
            echo json_encode($best_row); // Add "JSON_FORCE_OBJECT" if there shuld be identifiers but note these are "0" ... "max_num"
        } else {
            echo json_encode(0);
        }

        mysqli_close($con);
    } else {
        echo json_encode(0);
    }
?>
