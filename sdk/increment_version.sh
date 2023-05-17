cd sdk
head -n1 version.in > version.in.new
mv version.in.new version.in
oldnum=`cut -d ',' -f2 version.in`
newnum=$((oldnum + 1))
sed -i "s/.equ BUILD_VERSION,$oldnum\$/.equ BUILD_VERSION,$newnum/g" version.in

cat ../rust/Cargo.toml | grep "#xtal-16mhz" > /dev/null
ret=$?
if [ $ret -ne 0 ]; then
        echo ".equ XTAL_16MHZ,1" >> version.in
        cd ../rust/src
        echo "pub static BUILD_VERSION: u32 = (1 << 31) | $newnum;" > version.rs
else
        echo ".equ XTAL_16MHZ,0" >> version.in
        cd ../rust/src
        echo "pub static BUILD_VERSION: u32 = $newnum;" > version.rs
fi
