VER_FILE=version.in
echo -n " .equ BUILD_VERSION, " > $VER_FILE
echo 0x532e3156 >> $VER_FILE

#note: 0x522e3156 means V1.S
