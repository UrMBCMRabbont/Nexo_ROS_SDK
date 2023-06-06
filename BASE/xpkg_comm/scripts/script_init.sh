echo "*******************************"
echo "Copy udev rules for CAN_COM_HUB"
echo "///////////////////////////////"
FILE=/etc/udev/rules.d/CAN_COM_HUB.rules
if [ ! -f "$FILE" ];then
echo "Copy CAN2COM_HUB.rules"
sudo cp `rospack find xpkg_comm`/scripts/CAN_COM_HUB.rules /etc/udev/rules.d
else
echo "CAN2COM_HUB.rules existed,will cover file"
sudo rm -r /etc/udev/rules.d/CAN_COM_HUB.rules
sudo cp `rospack find xpkg_comm`/scripts/CAN_COM_HUB.rules /etc/udev/rules.d
fi
echo "*******************************"
echo "finish"
sudo systemctl stop nvgetty
sudo systemctl disable nvgetty
sudo service udev reload
sudo service udev restart
