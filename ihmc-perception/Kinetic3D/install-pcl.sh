#! /bin/bash

wget -O PCL-1.8.0-Linux.deb https://uc7d213d0e65e86c446488299ac0.dl.dropboxusercontent.com/cd/0/get/ANh0Ks7cw3BMXD9HfmMYqMfFwhR5RfswXFvdljQ5cFzEYLQNz8ekXMTB_L8drZKqmd2sUYuz-VDHjaiMWhHEInU6F-fTVt7OC9XIADZ-wCD49pxabVVAu9QgfQIHFYAf1-NWYIPMopl30wASXHl45rBPsyiccPmeEm_uPAjW6dIpiwSCk8l6pHaTFE2zAFiZYAA/file?_download_id=3872224248620053607604135383813739283182086978747748459593356756&_notify_domain=www.dropbox.com&dl=1
dpkg -i PCL-1.8.0-Linux.deb

apt-get install vtk6
ln -s /usr/lib/x86_64-linux-gnu/libvtkCommonCore-6.2.so /usr/lib/libvtkproj4.so

