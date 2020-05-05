#!/bin/bash
cd /Users/hansi/Sync/projects/2020/comemak/code/test_10_kresse
mkdir out
/usr/local/bin/gtimeout -s 9 2 /usr/local/bin/gphoto2 --auto-detect --capture-image
echo "Download all images.."
/usr/local/bin/gphoto2 --get-all-files

last=$(ls out | sort | tail -n 1)
seq=${last%.jpg}
seq=$(echo $seq | sed 's/^0*//')
seq=$(printf '%d' "$seq")
echo "Last sequence number: $seq"

for file in *.jpg
do
	seq=$((seq + 1))
	next=$(printf '%07d.jpg' "$seq")
	echo "$file -> out/$next"
	mv $file out/$next
done
echo seq=$next
/usr/local/bin/gphoto2 --delete-all-files --folder=/store_10000001