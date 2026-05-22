#!/usr/bin/env python3

import onnxruntime as ort
import numpy as np
import cv2
from numpy.typing import NDArray
import time

class PeopleSegmentation(object):
    def __init__(self, model_path: str):
        self.model = self._load_model_cpu(model_path)
        self.input_name = self.model.get_inputs()[0].name
        self.output_name = self.model.get_outputs()[0].name
    
    def _load_model_cpu(self, model_path: str) -> ort.InferenceSession:
        so = ort.SessionOptions()
        so.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        session = ort.InferenceSession(model_path, providers=['CPUExecutionProvider'])
        return session
    
    def _load_model(self, model_path: str) -> ort.InferenceSession:
        session = ort.InferenceSession(model_path, providers=['CUDAExecutionProvider', 'CPUExecutionProvider'])
        return session
    
    # def predict_segmentation(self, img: NDArray) -> NDArray:
    #     img_resized = cv2.resize(img, (512, 512))  # Resize to model's expected input
    #     img_normalized = img_resized / 255.0
    #     img_transposed = np.transpose(img_normalized, (2, 0, 1)).astype(np.float32)
    #     input_tensor = np.expand_dims(img_transposed, axis=0)
        
    #     # Run inference
    #     output = self.model.run([self.output_name], {self.input_name: input_tensor})[0]
        
    #     # Assuming output shape is (1, 1, H, W) or (1, H, W)
    #     mask = output[0]
    #     if mask.ndim == 3 and mask.shape[0] == 1:
    #         mask = mask[0]
        
    #     # Normalize and convert to 8-bit for saving
    #     mask = (mask > 0.5).astype(np.uint8) * 255
    #     mask = cv2.resize(mask, (img.shape[1], img.shape[0]), interpolation=cv2.INTER_NEAREST)
    #     return mask
    
    def predict_segmentation(self, img: NDArray) -> NDArray:
        # Preprocess input
        input_tensor = np.transpose(cv2.resize(img, (512, 512)).astype(np.float32) / 255.0, (2, 0, 1))
        input_tensor = np.transpose(input_tensor, (2, 0, 1))[np.newaxis, ...]

        # Inference
        output = self.model.run([self.output_name], {self.input_name: input_tensor})[0]
        mask = output[0][0] if output[0].ndim == 4 else output[0]  # shape: (H, W)

        # Binarize mask and resize to original image size
        mask = (mask > 0.5).astype(np.uint8) * 255
        mask_resized = cv2.resize(mask, (img.shape[1], img.shape[0]), interpolation=cv2.INTER_NEAREST)

        # Create red overlay where mask is present
        overlay = img.copy()
        overlay[mask_resized == 255] = [0, 0, 255]  # Red in BGR

        # Blend original image and overlay (optional alpha blending)
        alpha = 0.5
        blended = cv2.addWeighted(overlay, alpha, img, 1 - alpha, 0)

        return blended

    def predict_segmentation2(self, img: NDArray) -> NDArray:
        img_resized = cv2.resize(img, (512, 512))  # Resize to model's expected input
        img_normalized = img_resized / 255.0
        img_transposed = np.transpose(img_normalized, (2, 0, 1)).astype(np.float32)
        input_tensor = np.expand_dims(img_transposed, axis=0)
        

        output = self.model.run([self.output_name], {self.input_name: input_tensor})[0]
        # Assuming output shape is (1, 1, H, W) or (1, H, W)
        mask = output[0]
        if mask.ndim == 3 and mask.shape[0] == 1:
            mask = mask[0]
        
        # Binarize mask and resize to original image size
        mask = (mask > 0.5).astype(np.uint8) * 255
        mask_resized = cv2.resize(mask, (img.shape[1], img.shape[0]), interpolation=cv2.INTER_NEAREST)

        # Convert mask to 3 channels
        mask_rgb = cv2.cvtColor(mask_resized, cv2.COLOR_GRAY2BGR)

        # Create red overlay where mask is present
        overlay = img.copy()
        overlay[mask_resized == 255] = [0, 0, 255]  # Red in BGR

        # Blend original image and overlay (optional alpha blending)
        alpha = 0.5
        blended = cv2.addWeighted(overlay, alpha, img, 1 - alpha, 0)

        return blended

    def predict_segmentation_batch(self, img: NDArray) -> list[NDArray]:
        processed_batch = []
        original_shapes = []

        # Preprocess all images
        for i in range(2):
            original_shapes.append((img.shape[1], img.shape[0]))  # (width, height)
            img_resized = cv2.resize(img, (512, 512))
            img_normalized = img_resized / 255.0
            img_transposed = np.transpose(img_normalized, (2, 0, 1)).astype(np.float32)
            processed_batch.append(img_transposed)

        # Convert list to batch tensor
        input_tensor = np.stack(processed_batch, axis=0)  # Shape: (B, C, H, W)

        # Inference
        start_time = time.perf_counter()
        output_batch = self.model.run([self.output_name], {self.input_name: input_tensor})[0]
        stop_time = time.perf_counter()
        print(f"model estimated time for batch of {2}: {stop_time - start_time:.6f} secs")

        results = []
        for i in range(2):
            mask = output_batch[i]
            if mask.ndim == 3 and mask.shape[0] == 1:
                mask = mask[0]

            # Binarize and resize to original image size
            mask = (mask > 0.5).astype(np.uint8) * 255
            mask_resized = cv2.resize(mask, original_shapes[i], interpolation=cv2.INTER_NEAREST)

            # Convert mask to 3 channels
            mask_rgb = cv2.cvtColor(mask_resized, cv2.COLOR_GRAY2BGR)

            # Overlay mask on original image
            overlay = img.copy()
            overlay[mask_resized == 255] = [0, 0, 255]  # Red in BGR

            alpha = 0.5
            blended = cv2.addWeighted(overlay, alpha, img, 1 - alpha, 0)

            results.append(blended)

        return results

def test():
    model_path = "/workspaces/isaac_ros-dev/isaac_ros_assets/models/peoplesemsegformer/1/model.onnx"
    img = cv2.imread("image.png")
    if img is None:
        print("Failed to load image.")
        return
    
    people_seg = PeopleSegmentation(model_path)
    start_time = time.time()
    mask = people_seg.predict_segmentation2(img)
    stop_time = time.time()
    print(f"estimated time: {stop_time-start_time} secs")
    # cv2.imwrite("segmentation_mask.png", mask)
    print("Saved image mask.")

if __name__ == '__main__':
    test()
