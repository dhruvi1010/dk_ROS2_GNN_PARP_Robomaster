import cv2
import numpy as np
import tensorrt as trt
import pycuda.driver as cuda
import common  # This is assumed to have allocate_buffers() and do_inference()
# import pycuda.autoinit
import time

def demo_test3():
    model_path = "/workspaces/isaac_ros-dev/isaac_ros_assets/models/peoplesemsegformer/1/model.plan"
    original_img = cv2.imread("image.png")
    if original_img is None:
        print("Failed to load image.")
        return

    # Preprocess for model
    img = cv2.cvtColor(original_img, cv2.COLOR_BGR2RGB)
    img = cv2.resize(img, (512, 512))
    img = img.astype(np.float32) / 255.0
    img = np.transpose(img, (2, 0, 1))
    img = np.expand_dims(img, axis=0)
    img = np.ascontiguousarray(img)

    # Load TensorRT engine
    with open(model_path, 'rb') as f:
        engine_data = f.read()

    logger = trt.Logger(trt.Logger.WARNING)
    runtime = trt.Runtime(logger)
    engine = runtime.deserialize_cuda_engine(engine_data)
    context = engine.create_execution_context()

    #print("engine shape ", engine.get_tensor_shape(0))  # e.g. (1, 3, 512, 512) or (-1, 3, 512, 512)

    # Allocate buffers
    inputs, outputs, bindings, stream = common.allocate_buffers(engine)

    # Warm-up (optional but recommended for GPU inference)
    for _ in range(5):
        np.copyto(inputs[0].host, img.ravel())
        _ = common.do_inference(context, engine, bindings=bindings, inputs=inputs, outputs=outputs, stream=stream)

    # Actual timed inference
    start_time = time.perf_counter()

    np.copyto(inputs[0].host, img.ravel())
    output = common.do_inference(context, engine, bindings=bindings, inputs=inputs, outputs=outputs, stream=stream)

    end_time = time.perf_counter()
    inference_time_ms = (end_time - start_time) * 1000
    print(f"Inference time: {inference_time_ms:.2f} ms")

    # Run inference
    for i in range(1000):
        start_time = time.perf_counter()
        np.copyto(inputs[0].host, img.ravel())
        output = common.do_inference(context, engine, bindings=bindings, inputs=inputs, outputs=outputs, stream=stream)

        end_time = time.perf_counter()

        print(f"elapsed time: {end_time-start_time} s")
    result = np.array(output[0]).reshape((512, 512))

    # Resize original image
    original_resized = cv2.resize(original_img, (512, 512))

    # Create a red overlay where the mask is white (or 1.0)
    mask = (result > 0.5).astype(np.uint8)  # Thresholding if needed
    red_overlay = np.zeros_like(original_resized)
    red_overlay[:, :, 2] = mask * 255  # Set red channel only

    # Overlay red mask on original image
    blended = cv2.addWeighted(original_resized, 1.0, red_overlay, 0.5, 0)

    # Save the result
    cv2.imwrite("segmentation_overlay_red.png", blended)
    print("Saved overlay image as segmentation_overlay_red.png")

    # input("waiting for input")

    # Convert the result to a colored mask
    # mask = (result * 255).astype(np.uint8)
    # mask_colored = cv2.applyColorMap(mask, cv2.COLORMAP_JET)

    # # Resize original image if needed
    # original_resized = cv2.resize(original_img, (512, 512))

    # # Blend the original image with the mask (overlay)
    # blended = cv2.addWeighted(original_resized, 0.6, mask_colored, 0.4, 0)

    # # Save the result
    # cv2.imwrite("segmentation_overlay.png", blended)
    # print("Saved overlay image as segmentation_overlay.png")


if __name__ == "__main__":
    demo_test3()

