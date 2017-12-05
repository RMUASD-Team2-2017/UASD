<?php
    // Require DB config
    require('config.php');
    // Connect to database
    $con = mysqli_connect($mysql_host, $mysql_user, $mysql_pw, $mysql_db);
    if (!$con) {
        die("Connection failed: " . mysqli_connect_error());
    }

    $page = $_SERVER['PHP_SELF'];
    $sec = "2";

    echo '<!DOCTYPE html>
    <html>
        <head>
            <link rel="stylesheet" href="style.css">
            <link rel="stylesheet" href="table.css">
            <meta http-equiv="refresh" content="'.$sec.';URL='.$page.'">
        </head>
    <body>
    <div class="centerDiv">
    <center><a href="../">Request AED</a> - <a href="show_drones.php">Drones</a> - <b>Requests</b></center>
    <center><h1>AED Requests</h1></center>';

    /* SHOW UNAPPROVED */
    $sql = "SELECT * FROM `AED_requests` WHERE  `approved` =0 ORDER BY  `int_id` DESC";
    $result = mysqli_query($con, $sql);          //query
    echo '<h2>Awaiting approval or rejection</h2>';
    echo '<table class="customTableClass">
    <thead>
    <tr>
    <th>Request #</th>
    <th>Request ID</th>
    <th>Time and date</th>
    <th>Action</th>
    </tr>
    </thead>
    <tbody>';

    while($row = mysqli_fetch_array($result))
    {
        // if ($row['completed'] == 0) {
        //     echo '<tr style="background-color: #FF0000;">';
        //
        // } else {
        //     echo '<tr style="background-color: #90EE90;">';
        // }
        echo '<tr style="background-color: #FF0000;">';

        echo "<td>" . $row['int_id'] . "</td>";
        echo "<td>" . $row['request_id'] . "</td>";
        echo "<td>" . date('Y-m-d H:i:s', strtotime($row['time'])) . "</td>";
        echo '<td><a href="https://www.techgen.dk/AED/admin/approve_request.php?id='. $row['request_id'] . '&page=show_requests.php' .'">Approve</a> - <a href="https://www.techgen.dk/AED/admin/reject_request.php?id='. $row['request_id'] . '&page=show_requests.php' . '">Reject</a> - <a href="https://www.techgen.dk/AED/admin/show_request.php?id='. $row['request_id'] . '">See more</a></td>';
        echo "</tr>";
    }
    echo "</tbody>
    </table>";

    /* SHOW APPROVED BUT UNCOMPLETED */
    $sql = "SELECT * FROM `AED_requests` WHERE  `approved` !=0 AND `approved` !=2 AND `completed` =0 ORDER BY  `int_id` DESC";
    $result = mysqli_query($con, $sql);          //query
    echo '<h2>Awaiting completion</h2>';
    echo '<table class="customTableClass">
    <thead>
    <tr>
    <th>Request #</th>
    <th>Request ID</th>
    <th>Assigned UAV ID</th>
    <th>Time and date</th>
    <th>ETA</th>
    <th>Action</th>
    </tr>
    </thead>
    <tbody>';

    while($row = mysqli_fetch_array($result))
    {
        echo '<tr style="background-color: #FFFF00;">';

        echo "<td>" . $row['int_id'] . "</td>";
        echo "<td>" . $row['request_id'] . "</td>";
        if ($row['drone_id'] == 0) {
            echo "<td>None</td>";
        } else {
            echo "<td>" . $row['drone_id'] . "</td>";
        }
        echo "<td>" . date('Y-m-d H:i:s', strtotime($row['time'])) . "</td>";
        if ($row['eta'] == 0) {
            echo "<td>None</td>";
        } else {
            echo "<td>" . date('Y-m-d H:i:s', strtotime($row['eta'])) . "</td>";
        }
        echo '<td><a href="https://www.techgen.dk/AED/admin/show_request.php?id='. $row['request_id'] . '">See more</a></td>';
        echo "</tr>";
    }
    echo "</tbody>
    </table>";

    /* SHOW APPROVED AND COMPLETED */
    $sql = "SELECT * FROM `AED_requests` WHERE  `approved` !=0 AND `approved` !=2 and `completed` !=0 ORDER BY  `completed` DESC LIMIT 10";
    $result = mysqli_query($con, $sql);          //query
    echo '<h2>Completed</h2>';
    echo '<table class="customTableClass">
    <thead>
    <tr>
    <th>Request #</th>
    <th>Request ID</th>
    <th>Assigned UAV ID</th>
    <th>Time and date</th>
    <th>Action</th>
    </tr>
    </thead>
    <tbody>';

    while($row = mysqli_fetch_array($result))
    {
        echo '<tr style="background-color: #90EE90;">';

        echo "<td>" . $row['int_id'] . "</td>";
        echo "<td>" . $row['request_id'] . "</td>";
        if ($row['drone_id'] == 0) {
            echo "<td>None</td>";
        } else {
            echo "<td>" . $row['drone_id'] . "</td>";
        }
        echo "<td>" . date('Y-m-d H:i:s', strtotime($row['time'])) . "</td>";
        echo '<td><a href="https://www.techgen.dk/AED/admin/show_request.php?id='. $row['request_id'] . '">See more</a></td>';
        echo "</tr>";
    }
    echo "</tbody>
    </table>";
    echo '<p><i>showing the 10 latest completed requests</i></p>';

    /* SHOW REJECTED */
    $sql = "SELECT * FROM `AED_requests` WHERE `approved` =2 ORDER BY  `int_id` DESC";
    $result = mysqli_query($con, $sql);          //query
    echo '<h2>Rejected</h2>';
    echo '<table class="customTableClass">
    <thead>
    <tr>
    <th>Request #</th>
    <th>Request ID</th>
    <th>Time and date</th>
    <th>Action</th>
    </tr>
    </thead>
    <tbody>';

    while($row = mysqli_fetch_array($result))
    {
        echo '<tr style="background-color: #CCCCCC;">';

        echo "<td>" . $row['int_id'] . "</td>";
        echo "<td>" . $row['request_id'] . "</td>";
        echo "<td>" . date('Y-m-d H:i:s', strtotime($row['time'])) . "</td>";
        echo '<td><a href="https://www.techgen.dk/AED/admin/show_request.php?id='. $row['request_id'] . '">See more</a></td>';
        echo "</tr>";
    }
    echo "</tbody>
    </table>";

    echo '<br />';
    echo 'Updated: '.date('Y-m-d H:i:s', time()).' (refreshes every '.$sec.' seconds)';
    echo '</div>
    </body>
    </html>';

    mysqli_close($con);
?>
