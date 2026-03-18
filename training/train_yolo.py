#!/usr/bin/env python3
"""
YOLO Training Script for SensorFusion Project
===========================================

Advanced YOLO training with support for multiple architectures and datasets.
Integrates with automated dataset downloads from download-datasets.sh.
"""

import os
import sys
import yaml
import argparse
import logging
from pathlib import Path
from typing import Optional, Dict, Any

try:
    from ultralytics import YOLO
    import torch
    import cv2
    import numpy as np
    from tqdm import tqdm
except ImportError as e:
    print(f"❌ Missing dependencies: {e}")
    print("💡 Run: pip install ultralytics torch opencv-python")
    sys.exit(1)

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

class YOLOTrainer:
    """Advanced YOLO training class with comprehensive features."""
    
    def __init__(self, config_path: Optional[str] = None):
        self.base_dir = Path(__file__).parent.parent
        self.config_path = config_path
        self.config = self._load_config()
        self.model = None
        
    def _load_config(self) -> Dict[str, Any]:
        """Load training configuration."""
        if self.config_path and Path(self.config_path).exists():
            with open(self.config_path, 'r') as f:
                return yaml.safe_load(f)
        
        # Default configuration
        return {
            'model': {
                'architecture': 'yolov8n.pt',
                'input_size': 640,
                'classes': 80
            },
            'training': {
                'epochs': 100,
                'batch_size': 16,
                'learning_rate': 0.01,
                'patience': 10,
                'save_period': 10,
                'workers': 8
            },
            'data': {
                'dataset_path': 'prepared-training-data/dataset.yaml',
                'train_split': 0.8,
                'val_split': 0.2
            },
            'augmentation': {
                'hsv_h': 0.015,
                'hsv_s': 0.7,
                'hsv_v': 0.4,
                'degrees': 0.0,
                'translate': 0.1,
                'scale': 0.9,
                'shear': 0.0,
                'perspective': 0.0,
                'flipud': 0.0,
                'fliplr': 0.5,
                'mosaic': 1.0,
                'mixup': 0.0
            }
        }
    
    def setup_environment(self):
        """Setup training environment and verify prerequisites."""
        logger.info("🔧 Setting up training environment...")
        
        # Check CUDA availability
        if torch.cuda.is_available():
            device_name = torch.cuda.get_device_name(0)
            logger.info(f"🚀 CUDA available: {device_name}")
            logger.info(f"📊 GPU Memory: {torch.cuda.get_device_properties(0).total_memory / 1e9:.1f} GB")
        else:
            logger.warning("⚠️  CUDA not available - training will use CPU (slower)")
        
        # Verify dataset exists
        dataset_path = self.base_dir / self.config['data']['dataset_path']
        if not dataset_path.exists():
            logger.error(f"❌ Dataset not found: {dataset_path}")
            logger.info("💡 Run: ./download-datasets.sh to download training data")
            return False
            
        # Verify dataset structure
        with open(dataset_path, 'r') as f:
            dataset_config = yaml.safe_load(f)
            
        # Use dataset path as base if specified, otherwise use project base
        dataset_base = Path(dataset_config.get('path', self.base_dir))
        if not dataset_base.is_absolute():
            dataset_base = self.base_dir / dataset_base
            
        train_path = dataset_base / dataset_config['train']
        val_path = dataset_base / dataset_config['val'] 
        
        if not train_path.exists() or not val_path.exists():
            logger.error("❌ Dataset images not found")
            logger.info("💡 Run: ./download-datasets.sh and select option 1 or 3")
            return False
            
        # Check for images
        train_images = list(train_path.glob('*.jpg')) + list(train_path.glob('*.png'))
        val_images = list(val_path.glob('*.jpg')) + list(val_path.glob('*.png'))
        
        logger.info(f"📸 Training images: {len(train_images)}")
        logger.info(f"📸 Validation images: {len(val_images)}")
        
        if len(train_images) == 0:
            logger.error("❌ No training images found")
            return False
            
        return True
    
    def load_model(self, model_path: Optional[str] = None):
        """Load YOLO model for training."""
        model_path = model_path or self.config['model']['architecture']
        
        logger.info(f"🤖 Loading model: {model_path}")
        
        # Check if model exists locally, if not try to download
        model_file = self.base_dir / "model" / "yolo" / model_path
        if not model_file.exists():
            # Try to load from ultralytics (will download if needed)
            try:
                self.model = YOLO(model_path)
                logger.info(f"✅ Model loaded: {model_path}")
            except Exception as e:
                logger.error(f"❌ Failed to load model: {e}")
                return False
        else:
            self.model = YOLO(str(model_file))
            logger.info(f"✅ Model loaded from: {model_file}")
            
        return True
    
    def train(self, resume: bool = False, project: Optional[str] = None):
        """Start YOLO training."""
        if not self.model:
            logger.error("❌ No model loaded")
            return False
            
        # Setup paths
        dataset_path = str(self.base_dir / self.config['data']['dataset_path'])
        project_path = project or str(self.base_dir / "runs")
        
        logger.info("🚀 Starting YOLO training...")
        logger.info(f"📊 Dataset: {dataset_path}")
        logger.info(f"💾 Output: {project_path}")
        
        # Training parameters
        train_params = {
            'data': dataset_path,
            'epochs': self.config['training']['epochs'],
            'batch': self.config['training']['batch_size'],
            'imgsz': self.config['model']['input_size'],
            'lr0': self.config['training']['learning_rate'],
            'patience': self.config['training']['patience'],
            'save_period': self.config['training']['save_period'],
            'workers': self.config['training']['workers'],
            'project': project_path,
            'name': 'yolo_training',
            'resume': resume,
            'cache': True,
            'device': 0 if torch.cuda.is_available() else 'cpu'
        }
        
        # Add augmentation parameters
        for key, value in self.config['augmentation'].items():
            train_params[key] = value
        
        try:
            # Start training
            results = self.model.train(**train_params)
            
            logger.info("✅ Training completed successfully!")
            logger.info(f"📊 Results saved to: {results.save_dir}")
            
            return results
            
        except Exception as e:
            logger.error(f"❌ Training failed: {e}")
            return False
    
    def validate(self, model_path: Optional[str] = None):
        """Validate trained model."""
        if model_path:
            self.model = YOLO(model_path)
        
        if not self.model:
            logger.error("❌ No model loaded for validation")
            return False
            
        dataset_path = str(self.base_dir / self.config['data']['dataset_path'])
        
        logger.info("🔍 Validating model...")
        results = self.model.val(data=dataset_path)
        
        logger.info("✅ Validation completed!")
        return results
    
    def export_model(self, format: str = 'onnx', model_path: Optional[str] = None):
        """Export trained model to different formats."""
        if model_path:
            self.model = YOLO(model_path)
            
        if not self.model:
            logger.error("❌ No model loaded for export")
            return False
            
        logger.info(f"📤 Exporting model to {format.upper()}...")
        
        try:
            export_path = self.model.export(format=format)
            logger.info(f"✅ Model exported to: {export_path}")
            return export_path
        except Exception as e:
            logger.error(f"❌ Export failed: {e}")
            return False


def main():
    """Main training function."""
    parser = argparse.ArgumentParser(description='YOLO Training for SensorFusion')
    parser.add_argument('--config', type=str, help='Path to training config file')
    parser.add_argument('--model', type=str, default='yolov8n.pt', help='Model architecture')
    parser.add_argument('--epochs', type=int, default=100, help='Number of training epochs')
    parser.add_argument('--batch', type=int, default=16, help='Batch size')
    parser.add_argument('--imgsz', type=int, default=640, help='Image size')
    parser.add_argument('--lr', type=float, default=0.01, help='Learning rate')
    parser.add_argument('--resume', action='store_true', help='Resume training')
    parser.add_argument('--validate', action='store_true', help='Run validation only')
    parser.add_argument('--export', type=str, help='Export model (onnx, tensorrt, etc.)')
    parser.add_argument('--project', type=str, help='Project directory for results')
    
    args = parser.parse_args()
    
    # Initialize trainer
    trainer = YOLOTrainer(config_path=args.config)
    
    # Override config with command line arguments
    if args.epochs:
        trainer.config['training']['epochs'] = args.epochs
    if args.batch:
        trainer.config['training']['batch_size'] = args.batch
    if args.imgsz:
        trainer.config['model']['input_size'] = args.imgsz
    if args.lr:
        trainer.config['training']['learning_rate'] = args.lr
    
    # Setup environment
    if not trainer.setup_environment():
        logger.error("❌ Environment setup failed")
        return 1
    
    # Load model
    if not trainer.load_model(args.model):
        logger.error("❌ Model loading failed")
        return 1
    
    # Execute requested operation
    if args.validate:
        trainer.validate()
    elif args.export:
        trainer.export_model(format=args.export)
    else:
        # Train the model
        results = trainer.train(resume=args.resume, project=args.project)
        if not results:
            return 1
    
    logger.info("🎉 Operation completed successfully!")
    return 0


if __name__ == '__main__':
    sys.exit(main())