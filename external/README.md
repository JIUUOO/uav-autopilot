# External Dependencies

This directory contains external dependencies used by the UAV autopilot project.

Currently, this project uses:

```text
external/
└── YDLidar-SDK/
```

## YDLidar SDK

`YDLidar-SDK` is included as a Git submodule.

It provides the native SDK and Python binding required to use YDLidar sensors from Python.

---

## 1. Initialize Submodules

From the repository root:

```bash
git submodule update --init --recursive
```

If the submodule folder already exists and causes a clone error:

```bash
rm -rf external/YDLidar-SDK
git submodule update --init --recursive
```

---

## 2. Install Build Dependencies

```bash
sudo apt update
sudo apt install -y \
  cmake \
  build-essential \
  python3-pip \
  python3-dev \
  swig \
  pkg-config
```

---

## 3. Build and Install YDLidar SDK

```bash
cd external/YDLidar-SDK

mkdir -p build
cd build

cmake ..
make -j$(nproc)
sudo make install
sudo ldconfig
```

---

## 4. Install Python Binding

```bash
cd external/YDLidar-SDK

python3 setup.py build
sudo python3 setup.py install
```

Check installation:

```bash
python3 -c "import ydlidar; print('ydlidar OK')"
```

---

## 5. USB Permission

Check the LiDAR device path:

```bash
ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
```

Add the user to the `dialout` group:

```bash
sudo usermod -aG dialout $USER
```

Log out and log back in.

For temporary testing:

```bash
sudo chmod 666 /dev/ttyUSB0
```

Replace `/dev/ttyUSB0` with the actual device path.
