from flask import Flask, request, jsonify
import cv2
import numpy as np

app = Flask(__name__)

@app.route('/predict', methods=['POST'])
def predict():
    try:
        if 'image' not in request.files:
            return jsonify({"error": "No image part"}), 400
        
        file = request.files['image']
        img_bytes = file.read()
        
        # Convert bytes to OpenCV image
        nparr = np.frombuffer(img_bytes, np.uint8)
        img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
        
        if img is not None:
            # 1. Create window ONLY when image arrives
            window_name = "AI Prediction Mock"
            cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)
            
            # 2. Draw mock detection
            height, width, _ = img.shape
            cv2.rectangle(img, (int(width*0.2), int(height*0.2)), 
                          (int(width*0.8), int(height*0.8)), (0, 255, 0), 3)
            cv2.putText(img, "Detected: Object", (int(width*0.2), int(height*0.2)-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            
            # 3. Show the image
            cv2.imshow(window_name, img)
            # waitKey(500) gives the window half a second to render 
            # before returning the response to ROS2
            cv2.waitKey(500)
            
            print(f"Captured: {width}x{height} image. Displaying...")
        
        return jsonify({
                    "status": "success",
                    "message": "Detection completed",
                    "detections": [
                        {
                            "class": "robot",
                            "conf": 0.95,
                            "box": [120, 45, 320, 280]  # [x_min, y_min, x_max, y_max]
                        },
                        {
                            "class": "pallet",
                            "conf": 0.82,
                            "box": [400, 200, 550, 450]
                        }
                    ],
                    "timestamp": 1734444307.5
                })

    except Exception as e:
        print(f"Error: {e}")
        return jsonify({"error": str(e)}), 500

if __name__ == '__main__':
    print("Mock Server is listening on port 5000...")
    # 'threaded=True' is essential to prevent the Read Timeout
    app.run(host='0.0.0.0', port=5000, threaded=True)