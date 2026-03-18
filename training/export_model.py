#!/usr/bin/env python3
"""
Model Export Script for SensorFusion YOLO Models
==============================================

Exports trained YOLO models to various formats for deployment.
Supports ONNX, TensorRT, CoreML, and other deployment formats.
"""

import os
import sys
import time
import argparse
import logging
from pathlib import Path
from typing import Optional, List, Dict, Any

try:
    from ultralytics import YOLO
    import torch
    import onnxruntime as ort
except ImportError as e:
    print(f"❌ Missing dependencies: {e}")
    print("💡 Run: pip install ultralytics torch onnxruntime")
    sys.exit(1)

# Setup logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class ModelExporter:
    """YOLO model export and optimization class."""
    
    def __init__(self):
        self.base_dir = Path(__file__).parent.parent
        self.supported_formats = [
            'onnx', 'tensorrt', 'coreml', 'openvino', 'engine', 'ncnn', 
            'tflite', 'edgetpu', 'tfjs', 'paddle'
        ]
        
    def load_model(self, model_path: str) -> YOLO:
        """Load YOLO model from file."""
        model_path = Path(model_path)
        
        if not model_path.exists():
            # Try to find in common locations
            possible_paths = [
                self.base_dir / model_path,
                self.base_dir / "runs" / "detect" / "train" / "weights" / "best.pt",
                self.base_dir / "runs" / "detect" / "train2" / "weights" / "best.pt",
                self.base_dir / "model" / "yolo" / model_path
            ]
            
            for path in possible_paths:
                if path.exists():
                    model_path = path
                    break
            else:
                raise FileNotFoundError(f"Model not found: {model_path}")
        
        logger.info(f"🤖 Loading model: {model_path}")
        model = YOLO(str(model_path))
        logger.info(f"✅ Model loaded successfully")
        
        return model
    
    def export_model(self, 
                    model_path: str,
                    format: str = 'onnx',
                    output_dir: Optional[str] = None,
                    optimize: bool = True,
                    **kwargs) -> str:
        """Export model to specified format."""
        
        if format not in self.supported_formats:
            raise ValueError(f"Unsupported format: {format}. Supported: {self.supported_formats}")
        
        # Load model
        model = self.load_model(model_path)
        
        # Setup export parameters
        export_params = {
            'format': format,
            'optimize': optimize,
            **kwargs
        }
        
        logger.info(f"📤 Exporting model to {format.upper()}...")
        start_time = time.time()
        
        try:
            # Export the model
            exported_path = model.export(**export_params)
            
            export_time = time.time() - start_time
            logger.info(f"✅ Export completed in {export_time:.2f}s")
            logger.info(f"📁 Exported model: {exported_path}")
            
            # Move to output directory if specified
            if output_dir:
                output_dir = Path(output_dir)
                output_dir.mkdir(parents=True, exist_ok=True)
                
                new_path = output_dir / Path(exported_path).name
                Path(exported_path).rename(new_path)
                exported_path = str(new_path)
                logger.info(f"📁 Moved to: {exported_path}")
            
            return exported_path
            
        except Exception as e:
            logger.error(f"❌ Export failed: {e}")
            raise
    
    def benchmark_model(self, model_path: str, format: str = 'onnx') -> Dict[str, Any]:
        """Benchmark exported model performance."""
        
        logger.info(f"🏃 Benchmarking {format.upper()} model...")
        
        if format == 'onnx':
            return self._benchmark_onnx(model_path)
        elif format == 'tensorrt':
            return self._benchmark_tensorrt(model_path)
        else:
            logger.warning(f"⚠️  Benchmarking not implemented for {format}")
            return {}
    
    def _benchmark_onnx(self, model_path: str) -> Dict[str, Any]:
        """Benchmark ONNX model."""
        try:
            import numpy as np
            
            # Create ONNX Runtime session
            session = ort.InferenceSession(model_path)
            
            # Get input details
            input_details = session.get_inputs()[0]
            input_shape = input_details.shape
            input_name = input_details.name
            
            logger.info(f"📊 Input shape: {input_shape}")
            
            # Create dummy input
            if input_shape[0] == 'batch_size' or input_shape[0] is None:
                input_shape[0] = 1
            
            dummy_input = np.random.randn(*input_shape).astype(np.float32)
            
            # Warm up
            for _ in range(5):
                session.run(None, {input_name: dummy_input})
            
            # Benchmark
            num_runs = 100
            start_time = time.time()
            
            for _ in range(num_runs):
                outputs = session.run(None, {input_name: dummy_input})
            
            total_time = time.time() - start_time
            avg_time = total_time / num_runs
            fps = 1.0 / avg_time
            
            results = {
                'format': 'ONNX',
                'input_shape': input_shape,
                'avg_inference_time_ms': avg_time * 1000,
                'fps': fps,
                'total_runs': num_runs,
                'total_time_s': total_time
            }
            
            logger.info(f"⚡ Average inference time: {avg_time*1000:.2f}ms")
            logger.info(f"🎯 FPS: {fps:.1f}")
            
            return results
            
        except Exception as e:
            logger.error(f"❌ ONNX benchmark failed: {e}")
            return {}
    
    def _benchmark_tensorrt(self, model_path: str) -> Dict[str, Any]:
        """Benchmark TensorRT model."""
        try:
            # This would require TensorRT Python bindings
            logger.warning("⚠️  TensorRT benchmarking requires TensorRT Python bindings")
            return {}
        except Exception as e:
            logger.error(f"❌ TensorRT benchmark failed: {e}")
            return {}
    
    def optimize_for_deployment(self, 
                              model_path: str, 
                              target_platform: str = 'cpu',
                              output_dir: Optional[str] = None) -> Dict[str, str]:
        """Optimize model for specific deployment platform."""
        
        logger.info(f"🔧 Optimizing model for {target_platform}...")
        
        exported_models = {}
        
        if target_platform.lower() == 'cpu':
            # Export ONNX for CPU deployment
            onnx_path = self.export_model(
                model_path=model_path,
                format='onnx',
                output_dir=output_dir,
                optimize=True,
                simplify=True
            )
            exported_models['onnx'] = onnx_path
            
        elif target_platform.lower() == 'gpu':
            # Export TensorRT for GPU deployment
            try:
                tensorrt_path = self.export_model(
                    model_path=model_path,
                    format='tensorrt',
                    output_dir=output_dir,
                    optimize=True
                )
                exported_models['tensorrt'] = tensorrt_path
            except Exception as e:
                logger.warning(f"⚠️  TensorRT export failed: {e}")
                # Fallback to ONNX
                onnx_path = self.export_model(
                    model_path=model_path,
                    format='onnx',
                    output_dir=output_dir,
                    optimize=True
                )
                exported_models['onnx'] = onnx_path
                
        elif target_platform.lower() == 'mobile':
            # Export for mobile deployment
            formats = ['coreml', 'tflite']
            for fmt in formats:
                try:
                    path = self.export_model(
                        model_path=model_path,
                        format=fmt,
                        output_dir=output_dir,
                        optimize=True
                    )
                    exported_models[fmt] = path
                except Exception as e:
                    logger.warning(f"⚠️  {fmt.upper()} export failed: {e}")
                    
        elif target_platform.lower() == 'edge':
            # Export for edge devices
            try:
                tflite_path = self.export_model(
                    model_path=model_path,
                    format='tflite',
                    output_dir=output_dir,
                    optimize=True,
                    int8=True  # Quantization for edge devices
                )
                exported_models['tflite'] = tflite_path
            except Exception as e:
                logger.warning(f"⚠️  TFLite export failed: {e}")
        
        logger.info(f"✅ Optimization completed. Exported: {list(exported_models.keys())}")
        return exported_models
    
    def create_deployment_package(self, 
                                model_path: str,
                                output_dir: str,
                                include_demo: bool = True) -> str:
        """Create complete deployment package."""
        
        output_dir = Path(output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)
        
        logger.info(f"📦 Creating deployment package in {output_dir}...")
        
        # Export models for different platforms
        models_dir = output_dir / "models"
        models_dir.mkdir(exist_ok=True)
        
        exported_models = {}
        
        # Export common formats
        for fmt in ['onnx', 'tflite']:
            try:
                path = self.export_model(
                    model_path=model_path,
                    format=fmt,
                    output_dir=str(models_dir)
                )
                exported_models[fmt] = path
            except Exception as e:
                logger.warning(f"⚠️  {fmt.upper()} export failed: {e}")
        
        # Create deployment scripts
        if include_demo:
            self._create_demo_scripts(output_dir, exported_models)
        
        # Create requirements file
        self._create_requirements_file(output_dir)
        
        # Create README
        self._create_deployment_readme(output_dir, exported_models)
        
        logger.info(f"✅ Deployment package created: {output_dir}")
        return str(output_dir)
    
    def _create_demo_scripts(self, output_dir: Path, models: Dict[str, str]):
        """Create demo inference scripts."""
        demo_dir = output_dir / "demo"
        demo_dir.mkdir(exist_ok=True)
        
        # ONNX demo script
        if 'onnx' in models:
            onnx_demo = '''#!/usr/bin/env python3
"""ONNX Model Inference Demo"""

import cv2
import numpy as np
import onnxruntime as ort
from pathlib import Path

def run_inference(image_path, model_path):
    # Load model
    session = ort.InferenceSession(model_path)
    
    # Load and preprocess image
    img = cv2.imread(image_path)
    img = cv2.resize(img, (640, 640))
    img = img.astype(np.float32) / 255.0
    img = np.transpose(img, (2, 0, 1))
    img = np.expand_dims(img, axis=0)
    
    # Run inference
    outputs = session.run(None, {'images': img})
    
    print(f"Inference completed. Output shape: {outputs[0].shape}")
    return outputs

if __name__ == '__main__':
    import sys
    if len(sys.argv) != 3:
        print("Usage: python onnx_demo.py <image_path> <model_path>")
        sys.exit(1)
    
    run_inference(sys.argv[1], sys.argv[2])
'''
            
            with open(demo_dir / "onnx_demo.py", 'w') as f:
                f.write(onnx_demo)
    
    def _create_requirements_file(self, output_dir: Path):
        """Create requirements.txt for deployment."""
        requirements = [
            "numpy",
            "opencv-python", 
            "onnxruntime",
            "Pillow"
        ]
        
        with open(output_dir / "requirements.txt", 'w') as f:
            f.write('\n'.join(requirements))
    
    def _create_deployment_readme(self, output_dir: Path, models: Dict[str, str]):
        """Create deployment README."""
        readme_content = f'''# Model Deployment Package

This package contains exported YOLO models ready for deployment.

## Available Models

{chr(10).join(f"- **{fmt.upper()}**: `{Path(path).name}`" for fmt, path in models.items())}

## Quick Start

### Python Inference
```bash
# Install dependencies
pip install -r requirements.txt

# Run ONNX demo
python demo/onnx_demo.py <image_path> models/*.onnx
```

### Model Specifications
- Input size: 640x640
- Format: RGB
- Normalization: 0-1 range

## Integration Examples

### ONNX Runtime (Python)
```python
import onnxruntime as ort

session = ort.InferenceSession("models/model.onnx")
outputs = session.run(None, {{"images": preprocessed_image}})
```

### OpenCV DNN (C++)
```cpp
cv::dnn::Net net = cv::dnn::readNetFromONNX("models/model.onnx");
net.setInput(blob);
cv::Mat output = net.forward();
```

Generated on: {time.strftime("%Y-%m-%d %H:%M:%S")}
'''
        
        with open(output_dir / "README.md", 'w') as f:
            f.write(readme_content)


def main():
    """Main export function."""
    parser = argparse.ArgumentParser(description='YOLO Model Export Tool')
    parser.add_argument('model', help='Path to trained model (.pt file)')
    parser.add_argument('--format', choices=[
        'onnx', 'tensorrt', 'coreml', 'openvino', 'engine', 'ncnn',
        'tflite', 'edgetpu', 'tfjs', 'paddle'
    ], default='onnx', help='Export format')
    parser.add_argument('--output-dir', help='Output directory for exported model')
    parser.add_argument('--optimize', action='store_true', default=True, help='Optimize exported model')
    parser.add_argument('--benchmark', action='store_true', help='Benchmark exported model')
    parser.add_argument('--platform', choices=['cpu', 'gpu', 'mobile', 'edge'], 
                       help='Optimize for specific platform')
    parser.add_argument('--package', action='store_true', help='Create deployment package')
    
    args = parser.parse_args()
    
    try:
        exporter = ModelExporter()
        
        if args.platform:
            # Platform-specific optimization
            models = exporter.optimize_for_deployment(
                model_path=args.model,
                target_platform=args.platform,
                output_dir=args.output_dir
            )
            
            for fmt, path in models.items():
                logger.info(f"📁 {fmt.upper()}: {path}")
                
                if args.benchmark:
                    results = exporter.benchmark_model(path, fmt)
                    if results:
                        logger.info(f"📊 Benchmark results: {results}")
        
        elif args.package:
            # Create deployment package
            package_dir = exporter.create_deployment_package(
                model_path=args.model,
                output_dir=args.output_dir or 'deployment_package'
            )
            logger.info(f"📦 Deployment package: {package_dir}")
            
        else:
            # Single format export
            exported_path = exporter.export_model(
                model_path=args.model,
                format=args.format,
                output_dir=args.output_dir,
                optimize=args.optimize
            )
            
            if args.benchmark:
                results = exporter.benchmark_model(exported_path, args.format)
                if results:
                    logger.info(f"📊 Benchmark results: {results}")
        
        logger.info("🎉 Export completed successfully!")
        
    except Exception as e:
        logger.error(f"❌ Export failed: {e}")
        return 1
    
    return 0


if __name__ == '__main__':
    sys.exit(main())