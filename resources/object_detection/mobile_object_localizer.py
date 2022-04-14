# Load the model
import cv2
import tensorflow as tf
interpreter = tf.lite.Interpreter(model_path='object_detection/object_detection_mobile_object_localizer_v1_1_default_1.tflite')
interpreter.allocate_tensors()

# Get input and output tensors.
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

# Load image data with dtype=np.uint8
imgPath = 'object_detection/IMG_6297.png'
img = cv2.imread(imgPath)
input_data = [cv2.resize(img,input_details[0]['shape'][1:3])]
# The input data's shape should match input_details[0]['shape'], which is
# BATCH_SIZE x HEIGHT (192) x WIDTH (192) x CHANNELS (3)
interpreter.set_tensor(input_details[0]['index'], input_data)

interpreter.invoke()

# The function `get_tensor()` returns a copy of the tensor data.
# Use `tensor()` in order to get a pointer to the tensor.
output_dict = {
    'num_detections': int(interpreter.get_tensor(output_details[3]["index"])),
    'detection_classes': interpreter.get_tensor(output_details[1]["index"]).astype(np.uint8),
    'detection_boxes' : interpreter.get_tensor(output_details[0]["index"]),
    'detection_scores' : interpreter.get_tensor(output_details[2]["index"])
    }
