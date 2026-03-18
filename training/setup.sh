#!/bin/bash

set -e

echo "🎨 SensorFusion Training Environment Setup"
echo "========================================="

# Color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

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

# Get script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

print_status "Project root: $PROJECT_ROOT"
print_status "Training directory: $SCRIPT_DIR"

# Change to project root
cd "$PROJECT_ROOT"

# Check if main setup already ran
if [ ! -d "venv" ]; then
    print_status "Running main project setup first..."
    if [ -x "./setup.sh" ]; then
        ./setup.sh --python-only
    else
        print_error "Main setup.sh not found or not executable"
        exit 1
    fi
fi

# Activate virtual environment
print_status "Activating Python virtual environment..."
source venv/bin/activate

# Install training-specific dependencies
print_status "Installing training dependencies..."
pip install -r training/requirements.txt

# Check CUDA availability
print_status "Checking CUDA availability..."
python -c "
import torch
if torch.cuda.is_available():
    print(f'✅ CUDA available: {torch.cuda.get_device_name(0)}')
    print(f'💾 GPU memory: {torch.cuda.get_device_properties(0).total_memory / 1e9:.1f} GB')
else:
    print('⚠️  CUDA not available - training will use CPU')
"

# Check ultralytics installation
print_status "Verifying Ultralytics installation..."
python -c "from ultralytics import YOLO; print('✅ Ultralytics YOLO successfully imported')"

# Create necessary directories
print_status "Creating training directories..."
mkdir -p runs/{detect,segment,classify,pose}
mkdir -p training/experiments
mkdir -p training/logs
mkdir -p exported_models

# Check if training data exists
if [ ! -f "prepared-training-data/dataset.yaml" ]; then
    print_warning "Training dataset not found"
    echo ""
    echo "To set up training data, run one of:"
    echo "  1. ./download-datasets.sh  # Download pre-made datasets"
    echo "  2. python training/prepare_dataset.py --action create-sample  # Create sample data"
    echo "  3. python training/prepare_dataset.py --action convert-coco --coco-json <path> --images-dir <path>"
else
    print_success "Training dataset found: prepared-training-data/dataset.yaml"
fi

# Validate training scripts
print_status "Validating training scripts..."

for script in "train_yolo.py" "prepare_dataset.py" "export_model.py"; do
    if python -m py_compile "training/$script"; then
        print_success "✅ $script syntax OK"
    else
        print_error "❌ $script has syntax errors"
        exit 1
    fi
done

# Make scripts executable
chmod +x training/*.py

# Create quick test
print_status "Running quick functionality test..."
cd training

# Test dataset preparation
echo "Testing dataset preparation..."
if python prepare_dataset.py --action create-sample --num-train 5 --num-val 2 --output test-dataset >/dev/null 2>&1; then
    print_success "✅ Dataset preparation works"
    rm -rf test-dataset
else
    print_warning "⚠️  Dataset preparation test failed"
fi

# Test model loading
echo "Testing model loading..."
if python -c "from train_yolo import YOLOTrainer; trainer = YOLOTrainer(); trainer.load_model('yolov8n.pt')" >/dev/null 2>&1; then
    print_success "✅ Model loading works"
else
    print_warning "⚠️  Model loading test failed"
fi

cd "$PROJECT_ROOT"

# Display usage information
echo ""
print_success "🎉 Training environment setup completed!"
echo ""
echo "📚 Quick Start Guide:"
echo "  1. Prepare dataset:"
echo "     ./download-datasets.sh  # OR"
echo "     python training/prepare_dataset.py --action create-sample"
echo ""
echo "  2. Train model:"
echo "     python training/train_yolo.py"
echo ""
echo "  3. Export model:"
echo "     python training/export_model.py runs/detect/train/weights/best.pt"
echo ""
echo "🔧 Advanced usage:"
echo "  - Custom training: python training/train_yolo.py --config training/config/training.yaml"
echo "  - Resume training: python training/train_yolo.py --resume"
echo "  - Multi-format export: python training/export_model.py model.pt --platform mobile"
echo "  - Validation only: python training/train_yolo.py --validate"
echo ""
echo "📊 Monitoring:"
echo "  - Training results: ls runs/detect/train*/"
echo "  - TensorBoard: tensorboard --logdir runs/detect"
echo "  - Exported models: ls exported_models/"
echo ""
print_success "Happy training! 🚀"

echo ""