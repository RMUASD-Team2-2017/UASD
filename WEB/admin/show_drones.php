<?php
    // Require DB config
    require('config.php');
    // Connect to database
    $con = mysqli_connect($mysql_host, $mysql_user, $mysql_pw, $mysql_db);
    if (!$con) {
        die("Connection failed: " . mysqli_connect_error());
    }
    $sql = "SELECT * FROM `AED_drone_list`";
    $result = mysqli_query($con, $sql);          //query

    echo '<!DOCTYPE html>
    <html>
        <head>
            <link rel="stylesheet" href="style.css">
            <link rel="stylesheet" href="table.css">
        </head>
    <body>
    <div class="centerDiv">
    <center><a href="../">Request AED</a> - <b>Drones</b> - <a href="show_requests.php">Requests</a></center>
    <center><h1>AED UAVs</h1></center>';

    echo '<table class="customTableClass">
    <thead>
    <tr>
    <th>ID</th>
    <th>Name</th>
    <th>GCS position</th>
    <th>State</th>
    <th>Current position</th>
    <th>Target position</th>
    <th>Weather*</th>
    <th>Comment</th>
    </tr>
    </thead>
    <tbody>';

    while($row = mysqli_fetch_array($result))
    {
        echo "<tr>";
            echo "<td>" . $row['id'] . "</td>";
            echo "<td>" . $row['name'] . "</td>";
            echo "<td>" . $row['loc_lat'] . ", " . $row['loc_lng'] . "</td>";
            if ($row['state'] == 'idle') {
                echo '<td style="background-color: #90EE90;" >' . $row['state'] . "</td>";
            } elseif ($row['state'] == 'maintenance') {
                echo '<td style="background-color: #FFFF00;" >' . $row['state'] . "</td>";
            } elseif ($row['state'] == 'error') {
                echo '<td style="background-color: #FF0000;" >' . $row['state'] . "</td>";
            } elseif ($row['state'] == 'landed') {
                echo '<td style="background-color: #A9A9A9;" >' . $row['state'] . "</td>";
            } else {
                echo '<td style="background-color: #7CFC00;" >' . $row['state'] . "</td>";
            }
            if ($row['state'] == 'idle') {
                echo "<td>GCS</td>";
                echo "<td>None</td>";
            } else {
                echo "<td>" . $row['cur_lat'] . ", " . $row['cur_lng'] . "</td>";
                if ($row['state'] == 'maintenance' || $row['state'] == 'error') {
                    echo "<td>None</td>";
                } else {
                    echo "<td>" . $row['target_lat'] . ", " . $row['target_lng'] . "</td>";
                }
            }
            echo "<td>" . $row['weather'] . "</td>";
            echo "<td>" . $row['comment'] . "</td>";
        echo "</tr>";
    }
    echo "</tbody>
    </table>";

    echo "<i>* updated during pre-flight check</i>";
    echo '</div>
    </body>
    </html>';

    mysqli_close($con);
?>
