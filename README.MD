# SensorFusion Project

A comprehensive sensor fusion system combining Camera and LiDAR data for object detection and tracking, with integrated YOLO training capabilities.

**🎯 Ultra-lightweight repository**: Contains only source code and automated setup scripts. All datasets (~800MB) are downloaded separately via scripts.

## 🚀 Quick Start

### 1. Complete Setup (Recommended)
```bash
# Clone the repository (fast - no large files!)
git clone https://github.com/karthikpakala/SensorFusion.git
cd SensorFusion

# Run complete setup (Python env + datasets/models + C++ build)  
./setup.sh
```

### 2. Step-by-step Setup
```bash
# 1. Setup Python environment only
./setup.sh --python-only

# 2. Download datasets and models
./download-datasets.sh

# 3. Build C++ components (optional)
mkdir build && cd build && cmake .. && make
```

### 3. Manual Setup
```bash
# Create Python environment
python3 -m venv venv
source venv/bin/activate
pip install -r training/requirements.txt

# Download data (interactive)  
./download-datasets.sh

# Build C++ (optional)
mkdir build && cd build && cmake .. && make -j4
```

## 📊 Data-Free Repository Structure

This repository contains **ZERO large files** for maximum efficiency:

- ✅ **Repository size**: <50MB (lightning-fast clones)
- ✅ **All code included**: C++ source, Python scripts, configurations  
- ✅ **Automated downloads**: ~800MB of datasets downloaded as needed
- ✅ **Multiple data sources**: Flexible dataset management

### What's Included vs Downloaded
```
📁 SensorFusion/ (Repository)           📥 Downloaded Separately  
├── src/                 (included)     ├── prepared-training-data/ (~741MB)
├── include/             (included)     ├── example_dataset/        (~32KB) 
├── training/            (included)     ├── runs/                   (~51MB)
├── model/yolo/*.cfg     (included)     └── model/yolo/*.weights    (~248MB)
├── *.sh scripts         (included)
└── CMakeLists.txt       (included)
```

## 🛠️ Available Scripts

### Primary Scripts
- **`./setup.sh`** - Complete project setup (Python + datasets + C++)
- **`./download-datasets.sh`** - Download/setup training data & models
- **`./clean-data.sh`** - Clean up downloaded datasets to save space

### Setup Options
```bash
./setup.sh                    # Full setup
./setup.sh --python-only      # Python environment only  
./setup.sh --skip-datasets    # Skip dataset downloads
./setup.sh --skip-cpp         # Skip C++ build

./download-datasets.sh        # Interactive dataset downloader
./clean-data.sh              # Clean up datasets
```

## 📚 Usage Examples

### YOLO Training
```bash
source venv/bin/activate
python training/train_yolo.py

# View results
ls runs/train/exp/
```

### C++ Sensor Fusion
```bash
./build/SensorFusion
# or
cd build && ./SensorFusion
```

### Python Analysis
```bash
source venv/bin/activate  
python src/main.py
```

## 📂 Dataset Management

### Expected Structure (After Download)
```
prepared-training-data/
├── dataset.yaml            # YOLO dataset configuration
├── images/
│   ├── train/             # Training images (.jpg, .png)
│   └── val/               # Validation images
├── labels/ 
│   ├── train/             # Training labels (.txt, YOLO format)
│   └── val/               # Validation labels
└── README.md              # Dataset documentation

model/yolo/
├── coco.names              # Class names (included)
├── yolov3.cfg             # Model config (included)
├── yolov3-tiny.cfg        # Tiny model config (included)
├── yolov3.weights         # Model weights (downloaded)
└── yolov3-tiny.weights    # Tiny weights (downloaded)
```

### Adding Your Own Dataset
```bash
# 1. Prepare your data
cp your_images/* prepared-training-data/images/train/
cp your_labels/* prepared-training-data/labels/train/

# 2. Configure dataset.yaml
# Edit: prepared-training-data/dataset.yaml
# Set: nc (number of classes), names (class names)

# 3. Start training
python training/train_yolo.py
```

## 🔧 Development

### Repository Management
```bash
# Check what's downloaded
ls -la prepared-training-data/ model/yolo/ runs/

# Free up space
./clean-data.sh

# Re-download specific components  
./download-datasets.sh  # Choose option 1 for models only
```

### Building and Testing
```bash  
# C++ build
mkdir build && cd build
cmake .. && make -j$(nproc)
ctest  # Run C++ tests

# Python testing
pytest test/
```

## 📋 Requirements

- **System**: Linux/macOS/Windows
- **Python**: 3.8+ with pip
- **C++ Build**: CMake 3.16+ (optional)
- **Storage**: ~1GB free space for datasets
- **Network**: For downloading datasets
- **GPU**: CUDA optional (for training acceleration)

## 🎯 Benefits of This Architecture

✅ **Lightning-fast cloning** - No large files in git  
✅ **GitHub-friendly** - No repository size limits  
✅ **Flexible data sources** - Multiple download options  
✅ **Space efficient** - Download only what you need  
✅ **Team collaboration** - Easy sharing and setup  
✅ **CI/CD ready** - Small repo, fast builds

## 📖 Project Goals

# SensorFusion Project

A comprehensive sensor fusion system combining Camera and LiDAR data for object detection and tracking, with integrated YOLO training capabilities.

**🎯 Ultra-lightweight repository**: Contains only source code and automated setup scripts. All datasets are downloaded separately via scripts for maximum efficiency.

## 🚀 Quick Start

### 1. Complete Setup (Recommended)
```bash
# Clone the repository (ultra-fast - only 224KB!)
git clone https://github.com/karthikpakala/SensorFusion.git
cd SensorFusion

# Complete setup (Python env + datasets/models + C++ build)  
./setup.sh
```

### 2. Step-by-step Setup
```bash
# 1. Setup Python environment only
./setup.sh --python-only

# 2. Download datasets and models
./download-datasets.sh

# 3. Build C++ components (optional)
mkdir build && cd build && cmake .. && make
```

## 📊 Zero-Data Repository Architecture

This repository contains **ABSOLUTELY NO large files** for ultimate efficiency:

- ✅ **Repository size**: 224KB (lightning-fast clones)
- ✅ **All code included**: C++ source, Python scripts, configurations  
- ✅ **Automated downloads**: Datasets downloaded as needed
- ✅ **Multiple sources**: Flexible dataset management

### What's Included vs Downloaded
```
📁 SensorFusion/ (Repository - 224KB)     📥 Downloaded Separately  
├── src/                 (included)        ├── prepared-training-data/ (~varies)
├── include/             (included)        ├── runs/                   (~varies)
├── model/yolo/*.cfg     (included)        └── model/yolo/*.weights    (~280MB)
├── *.sh scripts         (included)
└── CMakeLists.txt       (included)
```

## 🛠️ Available Scripts

### Primary Scripts
- **`./setup.sh`** - Complete project setup (Python + datasets + C++)
- **`./download-datasets.sh`** - Interactive dataset and model downloader
- **`./clean-data.sh`** - Clean up downloaded data to save space

### Setup Options
```bash
./setup.sh                    # Full setup
./setup.sh --python-only      # Python environment only  
./setup.sh --skip-datasets    # Skip dataset downloads
./setup.sh --skip-cpp         # Skip C++ build

./download-datasets.sh        # Interactive data downloader
./clean-data.sh              # Clean up datasets
```

## 📚 Usage Examples

### YOLO Training
```bash
source venv/bin/activate
python -m ultralytics.yolo.v8.detect.train data=prepared-training-data/dataset.yaml model=yolov8n.pt

# View results
ls runs/detect/train*/
```

### C++ Sensor Fusion
```bash
./build/SensorFusion
```

### Python Analysis
```bash
source venv/bin/activate  
python src/main.py
```

## 📂 Dataset Management

### Expected Structure (After Download)
```
prepared-training-data/
├── dataset.yaml            # YOLO dataset configuration  
├── README.md              # Dataset documentation
├── images/
│   ├── train/             # Training images (.jpg, .png)
│   └── val/               # Validation images
└── labels/ 
    ├── train/             # Training labels (.txt, YOLO format)
    └── val/               # Validation labels

model/yolo/
├── coco.names              # Class names (included)
├── yolov3.cfg             # Model config (included)
├── yolov3-tiny.cfg        # Tiny model config (included)
├── yolov3.weights         # Model weights (downloaded)
└── yolov3-tiny.weights    # Tiny weights (downloaded)
```

### Adding Your Own Dataset
```bash
# 1. Setup structure
./download-datasets.sh  # Choose option 2 for structure

# 2. Add your data
cp your_images/* prepared-training-data/images/train/
cp your_labels/* prepared-training-data/labels/train/

# 3. Configure dataset.yaml
# Edit: prepared-training-data/dataset.yaml
# Set: nc (number of classes), names (class names)

# 4. Start training
python -m ultralytics.yolo.v8.detect.train data=prepared-training-data/dataset.yaml model=yolov8n.pt
```

## 🔧 Development

### Repository Management
```bash
# Check what's downloaded
ls -la prepared-training-data/ model/yolo/ runs/

# Free up space
./clean-data.sh

# Re-download components  
./download-datasets.sh  # Interactive options
```

### Building and Testing
```bash  
# C++ build
mkdir build && cd build
cmake .. && make -j$(nproc)
ctest  # Run C++ tests

# Python testing
pytest test/ || echo "Add your Python tests"
```

## 📋 Requirements

- **System**: Linux/macOS/Windows
- **Python**: 3.8+ with pip
- **C++ Build**: CMake 3.16+ (optional)
- **Storage**: ~1GB for full datasets (variable)
- **Network**: For downloading datasets
- **GPU**: CUDA optional (training acceleration)

## 🎯 Benefits of This Architecture

✅ **Ultra-fast cloning** - 224KB vs multi-GB repositories  
✅ **GitHub-perfect** - No size or LFS limitations  
✅ **Flexible data sources** - Academic, public, or custom datasets  
✅ **Space efficient** - Download only what you need  
✅ **CI/CD optimized** - Tiny repo, fast builds  
✅ **Team collaboration** - Easy sharing and setup  
✅ **Version control friendly** - Clean history, no binaries

## 📖 Project Goals

1. Load Camera and Lidar data and synchronize the data. - Done.

2. process Lidar Data to do the following:
    1. Read the Lidar data. 
    2. Filtering Point clouds.
    3. Process Lidar Points to segment various parts of the point clouds into road and objects
    4. Process the point clouds into different objects using clustering algorithms -  use KD-Tree 
    5. Create bounding boxes around the clustered objects and store them as needed. 

3. Process camera data to do the following
    1. Read camera image
    2. Convert to grey scale image
    3. Detect key points and key point descriptors
    4. Track objects using key points/descriptors

4. Combine Key points detection and Lidar BBs to create a sensor fusion model to detect and track objects
    1. Plot the ROI created for the PCL onto the image. 
    2. Identifty which key points fall into the PCL ROI and create a Camera and Lidar based sensor fusion model
    3. Create accuracy metrics to show the detection accuracy of various objects in the scene. 

    Make sure to only use Lidar or Camera data as needed. -- Optional. 

    Write up documentation for code distribution.

    Identify tools to visualize the output. 

Extended Goals:
    1. Integrate ML model for vehicle detection and classification.
    2. Create ROI around vehicles and fuse CAMERA/LIDAR data to create an object along with its classification.
    3. Integrate semantic segmented object detection pipeline.
    3. Create a separate pipeline to develop lane detection and tracking algorithms

5. Performance enhancements:
    1. Use smart pointers/move semantics to enable better usage of the memory as well as enable better performance
    2. Update segmentation function to be wayyyy more faster than it is now. look for improvements. 
    3. Update Lidar class to the best possible template class to be able to make it very versatile. 
    4. Use concurrency to enable better usage of resources and improve performance. 
    
    