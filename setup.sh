#!/bin/bash

set -e

echo "🚀 SensorFusion Complete Setup"
echo "=============================="

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

# Check system requirements
check_requirements() {
    print_status "Checking system requirements..."
    
    local missing_deps=()
    
    # Check Python
    if ! command -v python3 >/dev/null 2>&1; then
        missing_deps+=("python3")
    fi
    
    # Check pip  
    if ! command -v pip >/dev/null 2>&1 && ! command -v pip3 >/dev/null 2>&1; then
        missing_deps+=("pip")
    fi
    
    # Check git
    if ! command -v git >/dev/null 2>&1; then
        missing_deps+=("git")
    fi
    
    # Check cmake (for C++ components)
    if ! command -v cmake >/dev/null 2>&1; then
        print_warning "CMake not found - C++ components won't be available"
    fi
    
    if [ ${#missing_deps[@]} -gt 0 ]; then
        print_error "Missing dependencies: ${missing_deps[*]}"
        echo "Please install them using your package manager:"
        echo "  Ubuntu/Debian: sudo apt install python3 python3-pip git cmake"
        echo "  macOS: brew install python3 git cmake"
        exit 1
    fi
    
    print_success "System requirements satisfied"
}

# Setup Python virtual environment
setup_python_env() {
    print_status "Setting up Python virtual environment..."
    
    if [ ! -d "venv" ]; then
        python3 -m venv venv
        print_success "Virtual environment created"
    else
        print_success "Virtual environment already exists"
    fi
    
    # Activate virtual environment
    source venv/bin/activate
    
    # Upgrade pip
    python -m pip install --upgrade pip
    
    # Install Python dependencies
    print_status "Installing Python dependencies..."
    pip install torch torchvision torchaudio ultralytics opencv-python numpy matplotlib pyyaml tqdm
    print_success "Python dependencies installed"
    
    print_success "Python environment ready"
}

# Download datasets
setup_datasets() {
    echo ""
    print_status "Setting up datasets..."
    
    if [ -x "./download-datasets.sh" ]; then
        ./download-datasets.sh
    else
        print_warning "download-datasets.sh not found or not executable"
        chmod +x download-datasets.sh 2>/dev/null || true
        if [ -x "./download-datasets.sh" ]; then
            ./download-datasets.sh
        else
            print_error "Could not execute download-datasets.sh"
            echo "Please run it manually: ./download-datasets.sh"
        fi
    fi
}

# Build C++ components (optional)
build_cpp_components() {
    echo ""
    print_status "Building C++ components (optional)..."
    
    if command -v cmake >/dev/null 2>&1; then
        if [ -f "CMakeLists.txt" ]; then
            mkdir -p build
            cd build
            cmake ..
            make -j$(nproc 2>/dev/null || echo 4)
            cd ..
            print_success "C++ components built successfully"
        else
            print_warning "CMakeLists.txt not found - skipping C++ build"
        fi
    else
        print_warning "CMake not available - skipping C++ build"
    fi
}

# Validate setup
validate_setup() {
    echo ""
    print_status "Validating setup..."
    
    local errors=0
    
    # Check Python environment
    if [ -f "venv/bin/activate" ]; then
        print_success "✅ Python virtual environment ready"
    else
        print_error "❌ Python virtual environment missing"
        errors=$((errors + 1))
    fi
    
    # Check Python packages
    source venv/bin/activate 2>/dev/null || true
    if python -c "import torch, ultralytics" 2>/dev/null; then
        print_success "✅ Python dependencies ready"
    else
        print_warning "⚠️  Some Python dependencies missing"
        echo "   Run: pip install torch ultralytics"
    fi
    
    # Check C++ build
    if [ -f "build/SensorFusion" ]; then
        print_success "✅ C++ components ready"
    else
        print_warning "⚠️  C++ components not built (optional)"
    fi
    
    echo ""
    if [ $errors -eq 0 ]; then
        print_success "🎉 Setup completed successfully!"
        echo ""
        echo "📚 Next steps:"
        echo "   1. Activate environment: source venv/bin/activate"
        echo "   2. Download datasets: ./download-datasets.sh"
        echo "   3. Run training: python -m ultralytics.yolo.v8.detect.train data=prepared-training-data/dataset.yaml model=yolov8n.pt"
        echo "   4. Run C++ demo: ./build/SensorFusion"
    else
        print_warning "⚠️  Setup completed with warnings. Check messages above."
    fi
}

# Print usage information
print_usage() {
    echo ""
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Options:"
    echo "  --skip-datasets    Skip dataset download"
    echo "  --skip-cpp         Skip C++ build"
    echo "  --python-only      Only setup Python environment"
    echo "  --help             Show this help message"
    echo ""
}

# Parse command line arguments
skip_datasets=false
skip_cpp=false
python_only=false

while [[ $# -gt 0 ]]; do
    case $1 in
        --skip-datasets)
            skip_datasets=true
            shift
            ;;
        --skip-cpp)
            skip_cpp=true
            shift
            ;;
        --python-only)
            python_only=true
            shift
            ;;
        --help)
            print_usage
            exit 0
            ;;
        *)
            print_error "Unknown option: $1"
            print_usage
            exit 1
            ;;
    esac
done

# Main setup process
main() {
    echo ""
    check_requirements
    setup_python_env
    
    if [ "$python_only" = false ]; then
        if [ "$skip_datasets" = false ]; then
            setup_datasets
        fi
        
        if [ "$skip_cpp" = false ]; then
            build_cpp_components
        fi
    fi
    
    validate_setup
}

# Run main setup
main "$@"