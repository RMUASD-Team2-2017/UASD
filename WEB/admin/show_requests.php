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
    <center><a href="../">Request AED</a> - <a href="show_drones.php">Drones</a> - <b>Requests</b> - <a href="db.log"><i>Database exception log</i></a></center>
    <center><h1>AED Requests</h1></center>';

    //echo '<div style="width:50%; float:left;">';
    //echo '</div>';
    //echo '<div style="clear:both;"></div>';

    /* SHOW UNAPPROVED */
    $sql = "SELECT * FROM `AED_requests` WHERE  `approved` =0 AND `stopped` =0 ORDER BY  `int_id` DESC";
    $result = mysqli_query($con, $sql);          //query
    echo '<h2>Awaiting approval or rejection</h2>';
    echo '<table class="customTableClass">
    <thead>
    <tr>
    <th>Request #</th>
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
        echo "<td>" . date('Y-m-d ', strtotime($row['time'])) . "<b>" . date('H:i', strtotime($row['time'])) . "</b>" . date(':s', strtotime($row['time'])) . "</td>";
        echo '<td style="padding-top:3px;">';
        echo '<a href="https://www.techgen.dk/AED/admin/approve_request.php?id='. $row['request_id'] . '&page=show_requests.php' .'" class="confirmation"><img border="0" alt="Approve" src="approve_icon.png" width="24" height="24" style="margin-left:3px;"></a>';
        echo '<a href="https://www.techgen.dk/AED/admin/reject_request.php?id='. $row['request_id'] . '&page=show_requests.php' . '" class="confirmation"><img border="0" alt="Reject" src="reject_icon.png" width="24" height="24" style="margin-left:10px;"></a>';
        echo '<a href="https://www.techgen.dk/AED/admin/show_request.php?id='. $row['request_id'] . '"><img border="0" alt="See more" src="see-more_icon.png" width="24" height="24" style="margin-left:10px;"></a></td>';
        echo "</tr>";
    }
    echo "</tbody>
    </table>";

    /* SHOW APPROVED BUT UNCOMPLETED */
    $sql = "SELECT * FROM `AED_requests` WHERE  `approved` !=0 AND `approved` !=2 AND `completed` =0 AND `stopped` =0 ORDER BY  `int_id` DESC";
    $result = mysqli_query($con, $sql);          //query
    echo '<h2>Awaiting completion</h2>';
    echo '<table class="customTableClass">
    <thead>
    <tr>
    <th>Request #</th>
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
        if ($row['drone_id'] == 0) {
            echo "<td>None</td>";
        } else {
            echo "<td>" . $row['drone_id'] . "</td>";
        }
        echo "<td>" . date('Y-m-d ', strtotime($row['time'])) . "<b>" . date('H:i', strtotime($row['time'])) . "</b>" . date(':s', strtotime($row['time'])) . "</td>";
        if ($row['eta'] == 0) {
            echo "<td>None</td>";
        } else {
            echo "<td>" . date('Y-m-d H:i:s', strtotime($row['eta'])) . "</td>";
        }
        echo '<td style="padding-top:3px;"><a href="https://www.techgen.dk/AED/admin/show_request.php?id='. $row['request_id'] . '"><img border="0" alt="See more" src="see-more_icon.png" width="24" height="24" style="margin-left:3px;"></a></td>';
        echo "</tr>";
    }
    echo "</tbody>
    </table>";

    /* SHOW APPROVED AND COMPLETED */
    $sql = "SELECT * FROM `AED_requests` WHERE  `approved` !=0 AND `approved` !=2 and `completed` !=0 AND `stopped` =0 ORDER BY  `completed` DESC LIMIT 10";
    $result = mysqli_query($con, $sql);          //query
    echo '<h2>Completed</h2>';
    echo '<table class="customTableClass">
    <thead>
    <tr>
    <th>Request #</th>
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
        if ($row['drone_id'] == 0) {
            echo "<td>None</td>";
        } else {
            echo "<td>" . $row['drone_id'] . "</td>";
        }
        echo "<td>" . date('Y-m-d ', strtotime($row['time'])) . "<b>" . date('H:i', strtotime($row['time'])) . "</b>" . date(':s', strtotime($row['time'])) . "</td>";
        echo '<td style="padding-top:3px;"><a href="https://www.techgen.dk/AED/admin/show_request.php?id='. $row['request_id'] . '"><img border="0" alt="See more" src="see-more_icon.png" width="24" height="24" style="margin-left:3px;"></a></td>';
        echo "</tr>";
    }
    echo "</tbody>
    </table>";
    echo '<p><i>showing the 10 latest completed requests</i></p>';

    /* SHOW REJECTED */
    $sql = "SELECT * FROM `AED_requests` WHERE `approved` =2 AND `stopped` =0 ORDER BY  `int_id` DESC LIMIT 5";
    $result = mysqli_query($con, $sql);          //query
    echo '<h2>Rejected</h2>';
    echo '<table class="customTableClass">
    <thead>
    <tr>
    <th>Request #</th>
    <th>Time and date</th>
    <th>Action</th>
    </tr>
    </thead>
    <tbody>';

    while($row = mysqli_fetch_array($result))
    {
        echo '<tr style="background-color: #CCCCCC;">';

        echo "<td>" . $row['int_id'] . "</td>";
        echo "<td>" . date('Y-m-d ', strtotime($row['time'])) . "<b>" . date('H:i', strtotime($row['time'])) . "</b>" . date(':s', strtotime($row['time'])) . "</td>";
        echo '<td style="padding-top:3px;"><a href="https://www.techgen.dk/AED/admin/show_request.php?id='. $row['request_id'] . '"><img border="0" alt="See more" src="see-more_icon.png" width="24" height="24" style="margin-left:3px;"></a></td>';
        echo "</tr>";
    }
    echo "</tbody>
    </table>";
    echo '<p><i>showing the 5 latest rejected requests</i></p>';

    /* SHOW REJECTED */
    $sql = "SELECT * FROM `AED_requests` WHERE `stopped` !=0 ORDER BY  `int_id` DESC LIMIT 5";
    $result = mysqli_query($con, $sql);          //query
    echo '<h2>Stopped</h2>';
    echo '<table class="customTableClass">
    <thead>
    <tr>
    <th>Request #</th>
    <th>Time and date</th>
    <th>Reason</th>
    <th>Action</th>
    </tr>
    </thead>
    <tbody>';

    while($row = mysqli_fetch_array($result))
    {
        echo '<tr style="background-color: #CCCCCC;">';

        echo "<td>" . $row['int_id'] . "</td>";
        echo "<td>" . date('Y-m-d ', strtotime($row['time'])) . "<b>" . date('H:i', strtotime($row['time'])) . "</b>" . date(':s', strtotime($row['time'])) . "</td>";

        echo "<td>";
        if ($row['stopped'] == 1) {
            echo "Terminated";
        } elseif ($row['stopped'] == 2) {
            echo "Aborted";
        } elseif ($row['stopped'] == 3) {
            echo "Landed";
        }
        echo "</td>";

        echo '<td style="padding-top:3px;"><a href="https://www.techgen.dk/AED/admin/show_request.php?id='. $row['request_id'] . '"><img border="0" alt="See more" src="see-more_icon.png" width="24" height="24" style="margin-left:3px;"></a></td>';
        echo "</tr>";
    }
    echo "</tbody>
    </table>";
    echo '<p><i>showing the 5 latest stopped requests</i></p>';

    echo '<br />';
    echo 'Updated: '.date('Y-m-d H:i:s', time()).' (refreshes every '.$sec.' seconds)';
    echo '</div>';

    // jQuery not needed since the below script handles DOMs the old school way but there is nothing other jQuery would add for this page and therefore it is too heavy
    //echo '<script src="//ajax.googleapis.com/ajax/libs/jquery/1.8.1/jquery.min.js"></script>';

    echo '<script type="text/javascript">
        var elems = document.getElementsByClassName(\'confirmation\');
        var confirmIt = function (e) {
            if (!confirm(\'Are you sure?\')) e.preventDefault();
        };
        for (var i = 0, l = elems.length; i < l; i++) {
            elems[i].addEventListener(\'click\', confirmIt, false);
        }
        </script>';

    echo '</body>
    </html>';

    mysqli_close($con);
?>
