#!/bin/bash

set -e  # Exit on any error

echo "🚀 SensorFusion Dataset Downloader"
echo "==================================="
echo ""

# Color codes for better output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'  
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

print_status() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"  
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

print_header() {
    echo -e "${CYAN}$1${NC}"
}

# Function to check if command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Function to download with progress
download_with_progress() {
    local url="$1"
    local output="$2"
    
    if command_exists curl; then
        curl -L --progress-bar "$url" -o "$output"
    elif command_exists wget; then
        wget --progress=bar:force:noscroll "$url" -O "$output"
    else
        print_error "Neither curl nor wget found. Please install one of them."
        exit 1
    fi
}

# Create necessary directories
create_directories() {
    print_status "Creating directory structure..."
    mkdir -p prepared-training-data/{images/{train,val},labels/{train,val}}
    mkdir -p example_dataset/{images/{train,val},labels/{train,val}}
    mkdir -p model/yolo
    mkdir -p runs
    print_success "Directory structure created"
}

# Check what's already available
check_existing_data() {
    local found_something=false
    
    if [ -d "prepared-training-data/images" ] && [ "$(ls -A prepared-training-data/images 2>/dev/null)" ]; then
        local img_count=$(find prepared-training-data -name "*.jpg" 2>/dev/null | wc -l)
        print_success "Training images found: $img_count files"
        found_something=true
    fi
    
    if [ -d "model/yolo" ] && [ "$(ls -A model/yolo/*.weights 2>/dev/null)" ]; then  
        print_success "YOLO weights found in model/yolo/"
        found_something=true
    fi
    
    if [ -d "runs" ] && [ "$(ls -A runs 2>/dev/null)" ]; then
        print_success "Training results found in runs/"
        found_something=true
    fi
    
    if [ "$found_something" = true ]; then
        echo ""
        read -p "Data already exists. Do you want to re-download? (y/N): " -n 1 -r
        echo ""
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            print_status "Skipping download. Use './clean-data.sh' to force re-download."
            exit 0
        fi
    fi
}

# Download YOLO models  
download_yolo_models() {
    print_header "📦 Downloading YOLO Models"
    
    # YOLO v3 weights
    local yolov3_url="https://pjreddie.com/media/files/yolov3.weights"
    local yolov3_tiny_url="https://pjreddie.com/media/files/yolov3-tiny.weights"
    
    if [ ! -f "model/yolo/yolov3.weights" ]; then
        print_status "Downloading YOLOv3 weights (248MB)..."
        if download_with_progress "$yolov3_url" "model/yolo/yolov3.weights"; then
            print_success "YOLOv3 weights downloaded"
        else
            print_warning "Failed to download YOLOv3 weights"
        fi
    fi
    
    if [ ! -f "model/yolo/yolov3-tiny.weights" ]; then
        print_status "Downloading YOLOv3-tiny weights (34MB)..."
        if download_with_progress "$yolov3_tiny_url" "model/yolo/yolov3-tiny.weights"; then
            print_success "YOLOv3-tiny weights downloaded"
        else
            print_warning "Failed to download YOLOv3-tiny weights"
        fi
    fi
    
    # Download YOLOv8 model
    if command_exists python && python -c "import ultralytics" 2>/dev/null; then
        print_status "Downloading YOLOv8 model..."
        python -c "from ultralytics import YOLO; YOLO('yolov8n.pt')" || print_warning "Failed to download YOLOv8 model"
    fi
}

# Download sample datasets
download_sample_dataset() {
    print_header "📊 Setting Up Sample Dataset Structure"
    
    print_status "Creating sample dataset configuration..."
    
    # Create sample dataset.yaml
    cat > prepared-training-data/dataset.yaml << 'EOF'
# Dataset configuration for YOLO training
path: ../prepared-training-data  # dataset root dir
train: images/train  # train images (relative to 'path')
val: images/val      # val images (relative to 'path')
test:                # test images (optional)

# Classes (COCO dataset classes as example)
nc: 80  # number of classes
names: ['person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck', 'boat', 'traffic light',
        'fire hydrant', 'stop sign', 'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep', 'cow',
        'elephant', 'bear', 'zebra', 'giraffe', 'backpack', 'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee',
        'skis', 'snowboard', 'sports ball', 'kite', 'baseball bat', 'baseball glove', 'skateboard', 'surfboard',
        'tennis racket', 'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple',
        'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair', 'couch',
        'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop', 'mouse', 'remote', 'keyboard', 'cell phone',
        'microwave', 'oven', 'toaster', 'sink', 'refrigerator', 'book', 'clock', 'vase', 'scissors', 'teddy bear',
        'hair drier', 'toothbrush']
EOF
    
    # Create README for datasets
    cat > prepared-training-data/README.md << 'EOF'
# Training Dataset Directory

This directory should contain your training images and labels for YOLO model training.

## Directory Structure
```
prepared-training-data/
├── dataset.yaml          # Dataset configuration
├── README.md            # This file
├── images/
│   ├── train/           # Training images (.jpg, .png)
│   └── val/            # Validation images  
└── labels/
    ├── train/          # Training labels (.txt)
    └── val/            # Validation labels
```

## Label Format (YOLO)
Each `.txt` file should contain one line per object:
```
class_id x_center y_center width height
```
- All values normalized to [0, 1]
- Filename should match the image filename
- Example: `image001.jpg` → `image001.txt`

## Dataset Sources
1. **Custom Dataset**: Your own images and labels
2. **Public Datasets**: COCO, Pascal VOC, Open Images, etc.
3. **Generated Data**: Synthetic or augmented datasets

## Getting Started
1. Add your images to `images/train/` and `images/val/`
2. Add corresponding labels to `labels/train/` and `labels/val/`
3. Update `dataset.yaml` with your class names and count
4. Run training: `python training/train_yolo.py`

## Popular Dataset Downloads
- **COCO**: http://cocodataset.org/#download
- **Pascal VOC**: http://host.robots.ox.ac.uk/pascal/VOC/
- **Open Images**: https://storage.googleapis.com/openimages/web/index.html
EOF
    
    print_success "Sample dataset structure created"
}

# Main download function  
download_datasets() {
    echo ""
    print_status "🔽 What would you like to download?"
    echo "1) YOLO models only (recommended to start)"
    echo "2) Sample dataset structure only"  
    echo "3) Both models and sample structure"
    echo "4) Manual setup instructions"
    echo ""
    
    read -p "Select option (1-4) [1]: " choice
    choice=${choice:-1}
    
    case $choice in
        1)
            download_yolo_models
            ;;
        2)
            download_sample_dataset
            ;;
        3)
            download_yolo_models
            download_sample_dataset
            ;;
        4)
            print_manual_instructions
            ;;
        *)
            print_error "Invalid choice. Downloading YOLO models..."
            download_yolo_models
            ;;
    esac
}

print_manual_instructions() {
    echo ""
    print_header "📋 Manual Setup Instructions"
    echo ""
    echo "1. 📁 Dataset Structure (created by this script):"
    echo "   ./prepared-training-data/"
    echo "   ├── images/{train,val}/    # Your images here"
    echo "   ├── labels/{train,val}/    # YOLO labels here"  
    echo "   └── dataset.yaml           # Dataset configuration"
    echo ""
    echo "2. 📥 Download Training Data:"
    echo ""
    echo "   Option A - COCO Dataset:"
    echo "     wget http://images.cocodataset.org/zips/train2017.zip"
    echo "     wget http://images.cocodataset.org/zips/val2017.zip"
    echo "     # Extract and organize into prepared-training-data/"
    echo ""
    echo "   Option B - Pascal VOC:"  
    echo "     wget http://host.robots.ox.ac.uk/pascal/VOC/voc2012/VOCtrainval_11-May-2012.tar"
    echo ""
    echo "   Option C - Your Custom Dataset:"
    echo "     # Copy your images to prepared-training-data/images/{train,val}/"
    echo "     # Copy your labels to prepared-training-data/labels/{train,val}/"
    echo ""
    echo "3. 🤖 YOLO Models (handled by this script option 1):"
    echo "   - YOLOv3: 248MB"
    echo "   - YOLOv3-tiny: 34MB"
    echo "   - YOLOv8: Auto-downloaded by Ultralytics"
    echo ""
    echo "4. 🔧 Configure Training:"
    echo "   - Edit prepared-training-data/dataset.yaml"
    echo "   - Set number of classes (nc)"
    echo "   - Set class names"
    echo "   - Verify paths"
    echo ""
    echo "5. 🚀 Start Training:"
    echo "   python training/train_yolo.py"
    echo ""
}

verify_setup() {
    print_header "🔍 Verifying Setup"
    
    local errors=0
    local warnings=0
    
    # Check directory structure
    for dir in "prepared-training-data/images/train" "prepared-training-data/images/val" \
               "prepared-training-data/labels/train" "prepared-training-data/labels/val" \
               "model/yolo"; do
        if [ -d "$dir" ]; then
            print_success "✅ Directory exists: $dir"
        else
            print_error "❌ Missing directory: $dir"  
            errors=$((errors + 1))
        fi
    done
    
    # Check YOLO models
    if [ -f "model/yolo/yolov3.weights" ]; then
        local size=$(du -sh "model/yolo/yolov3.weights" | cut -f1)
        print_success "✅ YOLOv3 weights: $size"
    else
        print_warning "⚠️  YOLOv3 weights not found (run option 1 or 3)"
        warnings=$((warnings + 1))
    fi
    
    # Check training images
    local train_count=$(find prepared-training-data/images/train -name "*.jpg" -o -name "*.png" 2>/dev/null | wc -l)
    if [ "$train_count" -gt 0 ]; then
        print_success "✅ Training images: $train_count files"
    else
        print_warning "⚠️  No training images found - add your dataset"
        warnings=$((warnings + 1))
    fi
    
    # Check labels
    local label_count=$(find prepared-training-data/labels/train -name "*.txt" 2>/dev/null | wc -l)
    if [ "$label_count" -gt 0 ]; then
        print_success "✅ Training labels: $label_count files"
    else
        print_warning "⚠️  No training labels found - add your labels"
        warnings=$((warnings + 1))
    fi
    
    # Check dataset configuration
    if [ -f "prepared-training-data/dataset.yaml" ]; then
        print_success "✅ Dataset configuration ready"
    else
        print_warning "⚠️  Dataset configuration missing"
        warnings=$((warnings + 1))
    fi
    
    echo ""
    if [ $errors -eq 0 ]; then
        if [ $warnings -eq 0 ]; then
            print_success "🎉 Perfect! Everything is ready for training!"
            echo ""
            print_status "🚀 Next steps:"
            echo "   1. Activate environment: source venv/bin/activate"
            echo "   2. Install requirements: pip install ultralytics torch"
            echo "   3. Run training: python -m ultralytics.yolo.v8.detect.train data=prepared-training-data/dataset.yaml model=yolov8n.pt"
        else
            print_warning "⚠️  Setup completed with $warnings warning(s)."
            echo ""
            print_status "ℹ️  You can still proceed, but consider:"  
            echo "   - Adding training images to prepared-training-data/images/train/"
            echo "   - Adding labels to prepared-training-data/labels/train/"
            echo "   - Running: ./download-datasets.sh (option 1 for models)"
        fi
    else
        print_error "❌ Setup completed with $errors error(s). Please fix the issues above."
    fi
}

# Main execution
main() {
    create_directories
    check_existing_data
    download_datasets
    verify_setup
    
    echo ""
    print_status "📚 Additional Resources:"
    echo "   - Dataset setup: prepared-training-data/README.md"
    echo "   - Configuration: prepared-training-data/dataset.yaml"
    echo "   - Clean up data: ./clean-data.sh"
    echo "   - Complete setup: ./setup.sh"
    echo ""
    print_success "Dataset setup complete! 🎉"
}

# Run main function if script is executed directly
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    main "$@"
fi