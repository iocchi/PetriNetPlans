#!/bin/bash

bn=`dirname $(readlink -f "$0")`
echo "$bn"


jarfile1="$bn/.."
jarfile2="$bn/../lib/jarp"

if [ -r "$jarfile1/PNPjarp.jar" ]; then
	jarfile="$jarfile1/PNPjarp.jar"
else
	jarfile="$jarfile2/PNPjarp.jar"
fi

java -jar "$jarfile" "$@"
