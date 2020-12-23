#!/bin/sh

CURDIR=`pwd`
APPDIR="$(dirname -- "$(readlink -f -- "${0}")" )"

cd "$APPDIR"

# Set absolute path work around
sed -e "s,FULLPATH,$PWD,g" CANOpenAnalyzer.desktop > CANOpenAnalyzer.desktop.temp

cp CANOpenAnalyzer.desktop.temp ~/.local/share/applications/CANOpenAnalyzer.desktop
cp CANOpenAnalyzer.desktop.temp ~/Desktop/CANOpenAnalyzer.desktop
rm CANOpenAnalyzer.desktop.temp

echo "Installed CANOpenAnalyzer icons on menu and desktop !"

