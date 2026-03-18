#!/usr/bin/env python3
"""
Dataset Preparation Script for SensorFusion YOLO Training
========================================================

Prepares datasets for YOLO training with various input formats and augmentations.
Integrates with the automated download system from download-datasets.sh.
"""

import os
import sys
import json
import yaml
import shutil
import random
import argparse
import logging
from pathlib import Path
from typing import List, Dict, Tuple, Optional, Union

try:
    import cv2
    import numpy as np
    from PIL import Image
    from tqdm import tqdm
except ImportError as e:
    print(f"❌ Missing dependencies: {e}")
    print("💡 Run: pip install opencv-python pillow tqdm")
    sys.exit(1)

# Setup logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class DatasetPreparer:
    """Dataset preparation and conversion class."""
    
    def __init__(self, base_dir: Optional[Path] = None):
        self.base_dir = base_dir or Path(__file__).parent.parent
        self.supported_formats = ['.jpg', '.jpeg', '.png', '.bmp', '.tiff', '.tif']
        self.coco_classes = self._load_coco_classes()
        
    def _load_coco_classes(self) -> List[str]:
        """Load COCO class names."""
        coco_path = self.base_dir / "model" / "yolo" / "coco.names"
        if coco_path.exists():
            with open(coco_path, 'r') as f:
                return [line.strip() for line in f.readlines()]
        
        # Default COCO classes if file not found
        return [
            'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck',
            'boat', 'traffic light', 'fire hydrant', 'stop sign', 'parking meter', 'bench',
            'bird', 'cat', 'dog', 'horse', 'sheep', 'cow', 'elephant', 'bear', 'zebra',
            'giraffe', 'backpack', 'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee',
            'skis', 'snowboard', 'sports ball', 'kite', 'baseball bat', 'baseball glove',
            'skateboard', 'surfboard', 'tennis racket', 'bottle', 'wine glass', 'cup',
            'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple', 'sandwich', 'orange',
            'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair', 'couch',
            'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop', 'mouse',
            'remote', 'keyboard', 'cell phone', 'microwave', 'oven', 'toaster', 'sink',
            'refrigerator', 'book', 'clock', 'vase', 'scissors', 'teddy bear', 'hair drier', 'toothbrush'
        ]
    
    def create_yolo_structure(self, output_dir: Path):
        """Create standard YOLO dataset structure."""
        output_dir = Path(output_dir)
        
        # Create directories
        directories = [
            output_dir / 'images' / 'train',
            output_dir / 'images' / 'val',
            output_dir / 'labels' / 'train',
            output_dir / 'labels' / 'val'
        ]
        
        for dir_path in directories:
            dir_path.mkdir(parents=True, exist_ok=True)
            logger.info(f"📁 Created: {dir_path}")
    
    def convert_coco_to_yolo(self, 
                            coco_json: Path, 
                            images_dir: Path, 
                            output_dir: Path,
                            train_ratio: float = 0.8) -> bool:
        """Convert COCO format annotations to YOLO format."""
        try:
            with open(coco_json, 'r') as f:
                coco_data = json.load(f)
        except Exception as e:
            logger.error(f"❌ Failed to load COCO JSON: {e}")
            return False
        
        # Create YOLO structure
        self.create_yolo_structure(output_dir)
        
        # Create category mapping
        categories = {cat['id']: cat['name'] for cat in coco_data['categories']}
        class_mapping = {cat_id: idx for idx, cat_id in enumerate(categories.keys())}
        
        # Process images and annotations
        image_info = {img['id']: img for img in coco_data['images']}
        annotations_by_image = {}
        
        for ann in coco_data['annotations']:
            image_id = ann['image_id']
            if image_id not in annotations_by_image:
                annotations_by_image[image_id] = []
            annotations_by_image[image_id].append(ann)
        
        # Split into train/val
        image_ids = list(image_info.keys())
        random.shuffle(image_ids)
        split_point = int(len(image_ids) * train_ratio)
        train_ids = image_ids[:split_point]
        val_ids = image_ids[split_point:]
        
        logger.info(f"📊 Train images: {len(train_ids)}, Val images: {len(val_ids)}")
        
        # Process each split
        for split_name, image_ids_split in [('train', train_ids), ('val', val_ids)]:
            for image_id in tqdm(image_ids_split, desc=f"Processing {split_name}"):
                img_info = image_info[image_id]
                img_filename = img_info['file_name']
                img_path = images_dir / img_filename
                
                if not img_path.exists():
                    logger.warning(f"⚠️  Image not found: {img_path}")
                    continue
                
                # Copy image
                output_img_path = output_dir / 'images' / split_name / img_filename
                try:
                    shutil.copy2(img_path, output_img_path)
                except Exception as e:
                    logger.warning(f"⚠️  Failed to copy {img_filename}: {e}")
                    continue
                
                # Convert annotations
                if image_id in annotations_by_image:
                    yolo_annotations = []
                    img_width = img_info['width']
                    img_height = img_info['height']
                    
                    for ann in annotations_by_image[image_id]:
                        category_id = ann['category_id']
                        if category_id not in class_mapping:
                            continue
                        
                        class_idx = class_mapping[category_id]
                        bbox = ann['bbox']  # [x, y, width, height]
                        
                        # Convert to YOLO format (normalized center coordinates)
                        x_center = (bbox[0] + bbox[2] / 2) / img_width
                        y_center = (bbox[1] + bbox[3] / 2) / img_height
                        width = bbox[2] / img_width
                        height = bbox[3] / img_height
                        
                        yolo_annotations.append(f"{class_idx} {x_center:.6f} {y_center:.6f} {width:.6f} {height:.6f}")
                    
                    # Save YOLO annotation file
                    label_filename = Path(img_filename).stem + '.txt'
                    label_path = output_dir / 'labels' / split_name / label_filename
                    
                    with open(label_path, 'w') as f:
                        f.write('\\n'.join(yolo_annotations))
        
        # Create dataset.yaml
        dataset_config = {
            'path': str(output_dir.absolute()),
            'train': 'images/train',
            'val': 'images/val',
            'nc': len(categories),
            'names': list(categories.values())
        }
        
        yaml_path = output_dir / 'dataset.yaml'
        with open(yaml_path, 'w') as f:
            yaml.dump(dataset_config, f, default_flow_style=False)
        
        logger.info(f"✅ COCO to YOLO conversion completed!")
        logger.info(f"📄 Dataset config saved: {yaml_path}")
        
        return True
    
    def create_sample_dataset(self, 
                            output_dir: Path,
                            num_train: int = 100,
                            num_val: int = 20,
                            img_size: Tuple[int, int] = (640, 640)) -> bool:
        """Create a sample dataset with synthetic data for testing."""
        
        self.create_yolo_structure(output_dir)
        
        logger.info(f"🎨 Creating sample dataset: {num_train} train, {num_val} val images")
        
        # Create sample classes
        sample_classes = ['person', 'car', 'bicycle', 'dog', 'cat']
        
        def create_sample_image_and_label(split: str, idx: int):
            """Create a single sample image with random objects."""
            # Create random colored image
            img = np.random.randint(0, 255, (*img_size, 3), dtype=np.uint8)
            
            # Add some random shapes as "objects"
            annotations = []
            num_objects = random.randint(1, 5)
            
            for _ in range(num_objects):
                # Random class
                class_id = random.randint(0, len(sample_classes) - 1)
                
                # Random bounding box (normalized coordinates)
                x_center = random.uniform(0.1, 0.9)
                y_center = random.uniform(0.1, 0.9)
                width = random.uniform(0.05, 0.3)
                height = random.uniform(0.05, 0.3)
                
                # Ensure box is within image bounds
                x_center = max(width/2, min(1-width/2, x_center))
                y_center = max(height/2, min(1-height/2, y_center))
                
                annotations.append(f"{class_id} {x_center:.6f} {y_center:.6f} {width:.6f} {height:.6f}")
                
                # Draw rectangle on image (for visualization)
                x1 = int((x_center - width/2) * img_size[1])
                y1 = int((y_center - height/2) * img_size[0])
                x2 = int((x_center + width/2) * img_size[1])
                y2 = int((y_center + height/2) * img_size[0])
                
                color = tuple(random.randint(100, 255) for _ in range(3))
                cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
            
            # Save image
            img_filename = f"sample_{split}_{idx:04d}.jpg"
            img_path = output_dir / 'images' / split / img_filename
            cv2.imwrite(str(img_path), img)
            
            # Save label
            label_filename = f"sample_{split}_{idx:04d}.txt"
            label_path = output_dir / 'labels' / split / label_filename
            with open(label_path, 'w') as f:
                f.write('\\n'.join(annotations))
        
        # Create training images
        for i in tqdm(range(num_train), desc="Creating train images"):
            create_sample_image_and_label('train', i)
        
        # Create validation images
        for i in tqdm(range(num_val), desc="Creating val images"):
            create_sample_image_and_label('val', i)
        
        # Create dataset.yaml
        dataset_config = {
            'path': str(output_dir.absolute()),
            'train': 'images/train',
            'val': 'images/val',
            'nc': len(sample_classes),
            'names': sample_classes
        }
        
        yaml_path = output_dir / 'dataset.yaml'
        with open(yaml_path, 'w') as f:
            yaml.dump(dataset_config, f, default_flow_style=False)
        
        # Create README
        readme_content = f"""# Sample Dataset
        
This is a synthetically generated dataset for testing YOLO training.

## Statistics
- Training images: {num_train}
- Validation images: {num_val}
- Classes: {len(sample_classes)}
- Image size: {img_size}

## Classes
{chr(10).join(f'{i}: {cls}' for i, cls in enumerate(sample_classes))}

## Usage
```bash
python training/train_yolo.py --data {yaml_path}
```
"""
        
        with open(output_dir / 'README.md', 'w') as f:
            f.write(readme_content)
        
        logger.info(f"✅ Sample dataset created successfully!")
        logger.info(f"📄 Dataset config: {yaml_path}")
        
        return True
    
    def validate_dataset(self, dataset_path: Path) -> bool:
        """Validate YOLO dataset structure and integrity."""
        logger.info(f"🔍 Validating dataset: {dataset_path}")
        
        # Check if dataset.yaml exists
        yaml_path = dataset_path / 'dataset.yaml'
        if not yaml_path.exists():
            logger.error(f"❌ dataset.yaml not found: {yaml_path}")
            return False
        
        # Load dataset config
        try:
            with open(yaml_path, 'r') as f:
                config = yaml.safe_load(f)
        except Exception as e:
            logger.error(f"❌ Failed to load dataset.yaml: {e}")
            return False
        
        # Check required fields
        required_fields = ['train', 'val', 'nc', 'names']
        for field in required_fields:
            if field not in config:
                logger.error(f"❌ Missing field in dataset.yaml: {field}")
                return False
        
        # Check directories
        train_img_dir = dataset_path / config['train']
        val_img_dir = dataset_path / config['val']
        train_label_dir = dataset_path / config['train'].replace('images', 'labels')
        val_label_dir = dataset_path / config['val'].replace('images', 'labels')
        
        for name, dir_path in [
            ('Train images', train_img_dir),
            ('Val images', val_img_dir),
            ('Train labels', train_label_dir),
            ('Val labels', val_label_dir)
        ]:
            if not dir_path.exists():
                logger.error(f"❌ {name} directory not found: {dir_path}")
                return False
        
        # Count files
        train_images = list(train_img_dir.glob('*.jpg')) + list(train_img_dir.glob('*.png'))
        val_images = list(val_img_dir.glob('*.jpg')) + list(val_img_dir.glob('*.png'))
        train_labels = list(train_label_dir.glob('*.txt'))
        val_labels = list(val_label_dir.glob('*.txt'))
        
        logger.info(f"📊 Train: {len(train_images)} images, {len(train_labels)} labels")
        logger.info(f"📊 Val: {len(val_images)} images, {len(val_labels)} labels")
        
        # Check for missing labels
        missing_labels = 0
        for img_path in train_images + val_images:
            label_path = img_path.parent.parent / 'labels' / img_path.parent.name / f"{img_path.stem}.txt"
            if not label_path.exists():
                missing_labels += 1
        
        if missing_labels > 0:
            logger.warning(f"⚠️  {missing_labels} images have missing label files")
        
        # Validate a few random annotations
        sample_labels = random.sample(train_labels + val_labels, min(10, len(train_labels + val_labels)))
        for label_path in sample_labels:
            try:
                with open(label_path, 'r') as f:
                    lines = f.readlines()
                for line_num, line in enumerate(lines, 1):
                    parts = line.strip().split()
                    if len(parts) != 5:
                        logger.warning(f"⚠️  Invalid annotation format in {label_path}:{line_num}")
                        continue
                    
                    class_id = int(parts[0])
                    if class_id >= config['nc']:
                        logger.warning(f"⚠️  Invalid class ID {class_id} in {label_path}:{line_num}")
            except Exception as e:
                logger.warning(f"⚠️  Failed to validate {label_path}: {e}")
        
        logger.info("✅ Dataset validation completed!")
        return True


def main():
    """Main dataset preparation function."""
    parser = argparse.ArgumentParser(description='Dataset Preparation for YOLO Training')
    parser.add_argument('--action', choices=['create-sample', 'convert-coco', 'validate'], 
                       required=True, help='Action to perform')
    parser.add_argument('--output', type=str, default='prepared-training-data',
                       help='Output directory for prepared dataset')
    
    # For COCO conversion
    parser.add_argument('--coco-json', type=str, help='Path to COCO JSON annotation file')
    parser.add_argument('--images-dir', type=str, help='Directory containing COCO images')
    parser.add_argument('--train-ratio', type=float, default=0.8, help='Training split ratio')
    
    # For sample creation
    parser.add_argument('--num-train', type=int, default=100, help='Number of training images')
    parser.add_argument('--num-val', type=int, default=20, help='Number of validation images')
    parser.add_argument('--img-size', type=int, nargs=2, default=[640, 640], help='Image size [width height]')
    
    # For validation
    parser.add_argument('--dataset-dir', type=str, help='Dataset directory to validate')
    
    args = parser.parse_args()
    
    # Initialize preparer
    preparer = DatasetPreparer()
    
    if args.action == 'create-sample':
        output_dir = Path(args.output)
        success = preparer.create_sample_dataset(
            output_dir=output_dir,
            num_train=args.num_train,
            num_val=args.num_val,
            img_size=tuple(args.img_size)
        )
        
    elif args.action == 'convert-coco':
        if not args.coco_json or not args.images_dir:
            logger.error("❌ --coco-json and --images-dir are required for COCO conversion")
            return 1
        
        success = preparer.convert_coco_to_yolo(
            coco_json=Path(args.coco_json),
            images_dir=Path(args.images_dir),
            output_dir=Path(args.output),
            train_ratio=args.train_ratio
        )
        
    elif args.action == 'validate':
        dataset_dir = Path(args.dataset_dir or args.output)
        success = preparer.validate_dataset(dataset_dir)
    
    if success:
        logger.info("🎉 Operation completed successfully!")
        return 0
    else:
        logger.error("❌ Operation failed!")
        return 1


if __name__ == '__main__':
    sys.exit(main())