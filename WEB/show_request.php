<?php
    if ($_GET['id'] != '') {
        // Require DB config
        require('config.php');
        // Connect to database
        $con = mysqli_connect($mysql_host, $mysql_user, $mysql_pw, $mysql_db);
        if (!$con) {
            die("Connection failed: " . mysqli_connect_error());
        }

        $request_id = $_GET['id'];
        $sql = "SELECT * FROM `AED_smartphone_requests`  WHERE `id` = '$request_id'";
        $result = mysqli_query($con, $sql);          //query

        echo '<!DOCTYPE html>
        <html>
            <head>
                <link rel="stylesheet" href="style.css">
                <link rel="stylesheet" href="table.css">
            </head>
        <body>
        <div class="centerDiv">
        <center><a href="show_requests.php">⬑ Go Back</a></center>
        <center><h1>AED Request</h1><br /><h2>Request id: '.$request_id.'</h2></center>';

        if (mysqli_num_rows($result) != 0) {
            echo '<table class="customTableClass">
            <thead>
            <tr>
            <th>Request #</th>
            <th>Location</th>
            <th>Location accuracy</th>
            <th>Time and date</th>
            <th>GPS timestamp</th>
            <th>Altitude</th>
            <th>Altitude accuracy</th>
            </tr>
            </thead>
            <tbody>';
            $local_itr = 1;
            while($row = mysqli_fetch_array($result))
            {
                echo '<tr>';
                echo "<td>" . $local_itr . "</td>";
                echo "<td>" . $row['loc_lat'] . ", " . $row['loc_lng'] . "</td>";
                echo "<td>" . $row['loc_accuracy'] . "</td>";
                echo "<td>" . date('Y-m-d H:i:s', strtotime($row['req_time'])) . "</td>";
                echo "<td>" . $row['timestamp'] . "</td>";
                echo "<td>" . $row['altitude'] . "</td>";
                echo "<td>" . $row['altitude_accuracy'] . "</td>";
                echo "</tr>";
                $local_itr = $local_itr + 1;
            }
            echo "</tbody>
            </table>";
        } else {
            echo '<p style="color:red;">There is no entries concerning the specified request id.</p>';
        }

        echo '</div>
        </body>
        </html>';

        mysqli_close($con);
    } else {
        echo '<!DOCTYPE html>
        <html>
        <link rel="stylesheet" href="style.css">
        <link rel="stylesheet" href="table.css">
        <body>
        <div class="centerDiv">
        <center><h1>AED Request</h1><br /><h2 style="color:red;">No request id specified</h2></center>';

        echo '</div>
        </body>
        </html>';
    }

?>
