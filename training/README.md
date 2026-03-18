# SensorFusion Training Scripts

Comprehensive YOLO training pipeline for the SensorFusion project with automated dataset management and model export capabilities.

## 🎯 Features

- **Advanced YOLO Training**: YOLOv8 with customizable hyperparameters
- **Dataset Preparation**: COCO conversion, synthetic data generation, validation
- **Model Export**: Multiple formats (ONNX, TensorRT, CoreML, TFLite, etc.)
- **Automated Setup**: Integrates with project's download scripts
- **Comprehensive Validation**: Dataset and model validation tools
- **Deployment Ready**: Export optimization and benchmarking

## 🚀 Quick Start

### 1. Environment Setup
```bash
# From project root
./setup.sh  # Sets up Python env + downloads datasets

# OR manual setup
source venv/bin/activate
pip install -r training/requirements.txt
```

### 2. Prepare Dataset
```bash
# Option 1: Use downloaded dataset (recommended)
./download-datasets.sh  # Choose option 1 or 3

# Option 2: Create sample dataset for testing
python training/prepare_dataset.py --action create-sample --num-train 200 --num-val 40

# Option 3: Convert your COCO dataset
python training/prepare_dataset.py --action convert-coco --coco-json path/to/annotations.json --images-dir path/to/images
```

### 3. Train Model
```bash
# Basic training
python training/train_yolo.py

# Advanced training with custom parameters
python training/train_yolo.py --model yolov8s.pt --epochs 200 --batch 32 --imgsz 640

# Resume interrupted training
python training/train_yolo.py --resume
```

### 4. Export Model
```bash
# Export to ONNX (most common)
python training/export_model.py runs/detect/train/weights/best.pt --format onnx

# Platform-specific optimization
python training/export_model.py runs/detect/train/weights/best.pt --platform cpu
python training/export_model.py runs/detect/train/weights/best.pt --platform mobile

# Create deployment package
python training/export_model.py runs/detect/train/weights/best.pt --package
```

## 📁 File Structure

```
training/
├── train_yolo.py          # 🎯 Main training script
├── prepare_dataset.py     # 📊 Dataset preparation and conversion
├── export_model.py        # 📤 Model export and optimization
├── requirements.txt       # 📋 Python dependencies
├── README.md             # 📖 This documentation
└── config/               # 🔧 Configuration templates
    ├── training.yaml      # Training hyperparameters
    └── export.yaml        # Export settings
```

## 🔧 Configuration

### Training Configuration
Create `training/config/training.yaml`:
```yaml
model:
  architecture: yolov8n.pt
  input_size: 640
  classes: 80

training:
  epochs: 100
  batch_size: 16
  learning_rate: 0.01
  patience: 10
  workers: 8

data:
  dataset_path: prepared-training-data/dataset.yaml
  train_split: 0.8
  val_split: 0.2

augmentation:
  hsv_h: 0.015
  hsv_s: 0.7
  hsv_v: 0.4
  fliplr: 0.5
  mosaic: 1.0
```

## 🎓 Usage Examples

### Basic Training Workflow
```bash
# 1. Setup environment
./setup.sh
source venv/bin/activate

# 2. (Optional) Validate dataset
python training/prepare_dataset.py --action validate --dataset-dir prepared-training-data

# 3. Train model
python training/train_yolo.py --epochs 50 --batch 16

# 4. Export best model
python training/export_model.py runs/detect/train/weights/best.pt --format onnx --benchmark
```

### Custom Dataset Training
```bash
# 1. Convert your COCO dataset
python training/prepare_dataset.py --action convert-coco \
    --coco-json /path/to/your/annotations.json \
    --images-dir /path/to/your/images \
    --output custom-dataset

# 2. Train with custom dataset
python training/train_yolo.py --config custom-training.yaml

# 3. Validate results
python training/train_yolo.py --validate --model runs/detect/train/weights/best.pt
```

### Production Deployment
```bash
# 1. Platform-specific optimization
python training/export_model.py best.pt --platform gpu --output-dir production/models

# 2. Create deployment package
python training/export_model.py best.pt --package --output-dir production/deployment

# 3. Benchmark performance
python training/export_model.py production/models/best.onnx --benchmark
```

## 📊 Dataset Formats

### Supported Input Formats
- **YOLO**: Native format (images + txt annotations)
- **COCO**: JSON annotations with images
- **Custom**: Via preparation scripts

### Expected YOLO Structure
```
prepared-training-data/
├── dataset.yaml          # Dataset configuration
├── images/
│   ├── train/           # Training images (.jpg, .png)
│   └── val/             # Validation images  
└── labels/
    ├── train/           # Training labels (.txt, YOLO format)
    └── val/             # Validation labels
```

### YOLO Annotation Format
```
# Each line: class_id center_x center_y width height (normalized 0-1)
0 0.5 0.5 0.2 0.3
1 0.3 0.7 0.1 0.15
```

## 🚀 Advanced Features

### Hyperparameter Tuning
```bash
# Grid search (planned feature)
python training/train_yolo.py --tune --trials 50

# Custom learning rate schedule
python training/train_yolo.py --lr 0.001 --lrf 0.01
```

### Multi-GPU Training
```bash
# Use all available GPUs
python training/train_yolo.py --device 0,1,2,3

# Distributed training (advanced)
torchrun --nproc_per_node=4 training/train_yolo.py
```

### Model Ensemble
```bash
# Train multiple models
for model in yolov8n.pt yolov8s.pt yolov8m.pt; do
    python training/train_yolo.py --model $model --name ${model%.pt}_run
done

# Export ensemble (manual combination needed)
```

## 📈 Monitoring and Results

### Training Outputs
- **Logs**: Console output with training metrics
- **Results**: `runs/detect/train*/` - weights, plots, logs
- **Tensorboard**: `tensorboard --logdir runs/detect`
- **Weights**: Best and last model checkpoints

### Key Metrics
- **mAP@0.5**: Mean Average Precision at IoU 0.5
- **mAP@0.5:0.95**: Mean Average Precision across IoU thresholds
- **Precision/Recall**: Per-class performance metrics
- **Speed**: Inference time benchmarks

## 🐛 Troubleshooting

### Common Issues

**CUDA Out of Memory**
```bash
# Reduce batch size
python training/train_yolo.py --batch 8

# Use smaller model
python training/train_yolo.py --model yolov8n.pt
```

**Dataset Not Found**
```bash
# Re-download datasets
./download-datasets.sh

# Validate dataset structure
python training/prepare_dataset.py --action validate
```

**Export Fails**
```bash
# Install export dependencies
pip install onnx onnxruntime

# Try different format
python training/export_model.py model.pt --format tflite
```

### Performance Optimization

**Training Speed**
- Increase `--workers` for data loading
- Use `--cache ram` for small datasets
- Enable mixed precision with newer PyTorch

**Model Size vs Accuracy**
- `yolov8n.pt`: Smallest, fastest
- `yolov8s.pt`: Balanced
- `yolov8m.pt`: Larger, more accurate
- `yolov8l.pt`: Largest, best accuracy

## 🔗 Integration

### With Main SensorFusion Project
```bash
# Train model
python training/train_yolo.py

# Export for C++ integration
python training/export_model.py runs/detect/train/weights/best.pt --format onnx

# Move to model directory
mv best.onnx ../model/yolo/

# Update C++ code to use new model
```

### With Deployment Systems
```bash
# Create deployment package
python training/export_model.py model.pt --package --output-dir deployment

# Package contains:
# - models/ (ONNX, TFLite, etc.)
# - demo/ (inference scripts)
# - requirements.txt
# - README.md
```

## 📚 Additional Resources

- [Ultralytics YOLOv8 Docs](https://docs.ultralytics.com/)
- [YOLO Training Guide](https://docs.ultralytics.com/modes/train/)
- [Model Export Formats](https://docs.ultralytics.com/modes/export/)
- [SensorFusion Main README](../README.MD)

---

**💡 Pro Tip**: Start with the sample dataset to verify everything works, then move to your real data!