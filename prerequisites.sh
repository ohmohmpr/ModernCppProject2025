# download open3d
mkdir 3rdparty
wget https://github.com/isl-org/Open3D/releases/download/v0.18.0/open3d-devel-linux-x86_64-cxx11-abi-0.18.0.tar.xz
chmod +x open3d-devel-linux-x86_64-cxx11-abi-0.18.0.tar.xz
tar xf open3d-devel-linux-x86_64-cxx11-abi-0.18.0.tar.xz --transform 's/^open3d-devel-linux-x86_64-cxx11-abi-0.18.0/open3d/' -C 3rdparty
rm -rf open3d-devel-linux-x86_64-cxx11-abi-0.18.0.tar.xz

# sudo apt-get install libeigen3-dev