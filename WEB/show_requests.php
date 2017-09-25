<?php
    // Require DB config
    require('config.php');
    // Connect to database
    $con = mysqli_connect($mysql_host, $mysql_user, $mysql_pw, $mysql_db);
    if (!$con) {
        die("Connection failed: " . mysqli_connect_error());
    }
    $sql = "SELECT * FROM `AED_requests` ORDER BY  `int_id` DESC";
    $result = mysqli_query($con, $sql);          //query

    echo '<!DOCTYPE html>
    <html>
    <link rel="stylesheet" href="style.css">
    <link rel="stylesheet" href="table.css">
    <body>
    <div class="centerDiv">
    <center><h1>AED Requests</h1></center>';

    echo '<table class="customTableClass">
    <thead>
    <tr>
    <th>Request #</th>
    <th>Request ID</th>
    <th>Assigned UAV ID</th>
    <th>Time and date</th>
    <th>ETA</th>
    <th>Link</th>
    </tr>
    </thead>
    <tbody>';

    while($row = mysqli_fetch_array($result))
    {
        if ($row['completed'] == 0) {
            echo '<tr style="background-color: #FF0000;">';

        } else {
            echo '<tr style="background-color: #90EE90;">';
        }
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
        echo '<td><a href="https://www.techgen.dk/AED/show_request.php?id='. $row['request_id'] . '">See more</a></td>';
        echo "</tr>";
    }
    echo "</tbody>
    </table>";

    echo '</div>
    </body>
    </html>';

    mysqli_close($con);
?>
