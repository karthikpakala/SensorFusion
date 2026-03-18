#!/bin/bash

echo "🧹 SensorFusion Data Cleanup"
echo "============================"

# Color codes
RED='\033[0;31m'
YELLOW='\033[1;33m'
GREEN='\033[0;32m'
NC='\033[0m'

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check what can be cleaned
echo ""
echo "This script will remove downloaded datasets and training results:"
echo ""

total_size=0

# Check for datasets
if [ -d "prepared-training-data" ] && [ "$(ls -A prepared-training-data 2>/dev/null)" ]; then
    size=$(du -sh prepared-training-data 2>/dev/null | cut -f1 || echo "0")
    echo "📁 Training data: $size"
    total_size=$((total_size + $(du -sm prepared-training-data 2>/dev/null | cut -f1 || echo 0)))
fi

if [ -d "example_dataset" ] && [ "$(ls -A example_dataset 2>/dev/null)" ]; then
    size=$(du -sh example_dataset 2>/dev/null | cut -f1 || echo "0")
    echo "📁 Example dataset: $size"
    total_size=$((total_size + $(du -sm example_dataset 2>/dev/null | cut -f1 || echo 0)))
fi

if [ -d "runs" ] && [ "$(ls -A runs 2>/dev/null)" ]; then
    size=$(du -sh runs 2>/dev/null | cut -f1 || echo "0")
    echo "📁 Training results: $size"
    total_size=$((total_size + $(du -sm runs 2>/dev/null | cut -f1 || echo 0)))
fi

# Check for model weights (but keep configs)
if find model/yolo -name "*.weights" -o -name "*.pt" 2>/dev/null | grep -q .; then
    size=$(find model/yolo -name "*.weights" -o -name "*.pt" -exec du -ch {} + 2>/dev/null | tail -1 | cut -f1 || echo "0")
    echo "📁 YOLO model weights: $size"
    total_size=$((total_size + $(find model/yolo -name "*.weights" -o -name "*.pt" -exec du -sm {} + 2>/dev/null | awk '{sum+=$1} END {print sum}' || echo 0)))
fi

# Check for archives
if ls *.tar.gz *.zip 2>/dev/null | grep -q .; then
    for file in *.tar.gz *.zip; do
        if [ -f "$file" ]; then
            size=$(du -sh "$file" 2>/dev/null | cut -f1 || echo "0")
            echo "📦 Archive: $file ($size)"
            total_size=$((total_size + $(du -sm "$file" 2>/dev/null | cut -f1 || echo 0)))
        fi
    done
fi

echo ""
if [ $total_size -gt 0 ]; then
    echo "💾 Total space to free: ${total_size}MB"
    echo ""
    
    print_warning "This will permanently delete all downloaded datasets!"
    read -p "Are you sure you want to continue? (y/N): " -n 1 -r
    echo ""
    
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        echo ""
        echo "🗑️  Removing datasets..."
        
        # Remove dataset directories
        for dir in prepared-training-data example_dataset runs; do
            if [ -d "$dir" ]; then
                rm -rf "$dir"
                print_success "Removed $dir"
            fi
        done
        
        # Remove model weights but keep config files
        if [ -d "model/yolo" ]; then
            find model/yolo -name "*.weights" -o -name "*.pt" -delete 2>/dev/null || true
            print_success "Removed model weights (kept config files)"
        fi
        
        # Remove archives
        for pattern in "*.tar.gz" "*.zip" "*.tar.bz2" "*.7z"; do
            for file in $pattern; do
                if [ -f "$file" ]; then
                    rm -f "$file"
                    print_success "Removed $file"
                fi
            done
        done 2>/dev/null || true
        
        # Recreate empty directories with proper structure
        mkdir -p model/yolo
        mkdir -p prepared-training-data/{images/{train,val},labels/{train,val}}
        mkdir -p example_dataset/{images/{train,val},labels/{train,val}}
        mkdir -p runs
        
        echo ""
        print_success "✅ Cleanup completed!"
        echo ""
        echo "📥 To re-download datasets, run:"
        echo "   ./download-datasets.sh"
        
    else
        echo ""
        print_success "Cleanup cancelled."
    fi
else
    print_success "✨ No datasets found to clean up."
    echo ""
    echo "📥 To download datasets, run:"
    echo "   ./download-datasets.sh"
fi

echo ""