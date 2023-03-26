cd sdk
oldnum=`cut -d ',' -f2 version.in`
newnum=$((oldnum + 1))
sed -i "s/$oldnum\$/$newnum/g" version.in