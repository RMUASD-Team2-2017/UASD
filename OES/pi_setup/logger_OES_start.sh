#!/bin/sh
DATE=$(date +"%Y%m%d%H%M%S")
FILENAME="OES_"$DATE".log"
echo $FILENAME
touch $FILENAME

source tmp_test.sh >> $FILENAME 
