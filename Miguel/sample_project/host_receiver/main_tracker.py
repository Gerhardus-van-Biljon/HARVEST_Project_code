import cv2
import numpy as np
import pygetwindow as gw
import mss
from ultralytics import YOLO

#Loads the calibration data of the focal length and central point and unpacks them 
calib_data = np.load('camera_calib.npz')
camera_matrix = calib_data['camera_matrix']
dist_coefs = calib_data['dist_coefs']
fx = float(calib_data['fx'])
fy = float(calib_data['fy'])
cx = float(calib_data['cx'])
cy = float(calib_data['cy'])

print(f"[+] Loaded Calibration Matrix: fx={fx:.1f}, fy={fy:.1f}, cx={cx:.1f}, cy={cy:.1f}")

# Loading the weights of the strawberry detection to the model
model = YOLO('best.pt')

# Establishes a physical refrence on all strawberries so the depth can be calulated regardless
# of the classification label 
REAL_STRAWBERRY_WIDTH_MM = 35.0

# Map your custom trained classes directly
REAL_WIDTHS_MM = {
    'Ripe': REAL_STRAWBERRY_WIDTH_MM,
    'Semiripe': REAL_STRAWBERRY_WIDTH_MM,
    'Unripe': REAL_STRAWBERRY_WIDTH_MM,
    'ripe': REAL_STRAWBERRY_WIDTH_MM,
    'semiripe': REAL_STRAWBERRY_WIDTH_MM,
    'unripe': REAL_STRAWBERRY_WIDTH_MM,
    'strawberry': REAL_STRAWBERRY_WIDTH_MM
}

TARGET_KEYWORD = "ESP32-S3"
apply_undistort = False  # Set to False by default to inspect raw feed first

def find_target_window(keyword):
    for w in gw.getAllWindows():
        if keyword.lower() in w.title.lower():
            return w
    return None

def main():
    global apply_undistort
    win = find_target_window(TARGET_KEYWORD)
    if not win:
        print(f"[-] Window containing '{TARGET_KEYWORD}' not found. Make sure Edge/Chrome is open.")
        return

    if win.isMinimized:
        win.restore()

    # Fixes the constant screen duplication
    # Automatically position the stream browser window on the LEFT half of the screen
    try:
        win.activate()
        win.moveTo(0, 0)
        win.resizeTo(960, 1040)
    except Exception:
        pass

    print(f"[+] Tracking window: '{win.title}'")
    print("    [Press 'u' in the OpenCV window to toggle Undistortion ON/OFF]")
    print("    [Press 'ESC' to Exit]")

    # Create the OpenCV display window and force it to the RIGHT half of the screen
    cv2.namedWindow("ESP32-S3 Vision Pipeline", cv2.WINDOW_NORMAL)
    cv2.moveWindow("ESP32-S3 Vision Pipeline", 960, 0)
    cv2.resizeWindow("ESP32-S3 Vision Pipeline", 960, 1040)

    with mss.mss() as sct:
        new_cam_matrix = None

        while True:
            # Capture browser window bounds strictly on the left screen
            monitor = {
                "top": max(0, win.top),
                "left": max(0, win.left),
                "width": max(100, win.width),
                "height": max(100, win.height)
            }

            sct_img = sct.grab(monitor)
            full_frame = np.array(sct_img)
            full_frame = cv2.cvtColor(full_frame, cv2.COLOR_BGRA2BGR)

            h_full, w_full = full_frame.shape[:2]

            # Crop browser navigation bar/tabs so the video feed isnt clutterd 
            crop_top = int(h_full * 0.12)
            crop_bottom = int(h_full * 0.95)
            crop_left = int(w_full * 0.02)
            crop_right = int(w_full * 0.98)

            frame = full_frame[crop_top:crop_bottom, crop_left:crop_right]
            h, w = frame.shape[:2]

            if h <= 0 or w <= 0:
                frame = full_frame
                
# !!!! End of screen capture code and duplication issue solution !!!


            #Assists in removing fisheye bending which allows straight lines in the physical world to stay 
            #Straight when the image is taken preventing object from being bent along the frames edge
            if apply_undistort:
                if new_cam_matrix is None:
                    new_cam_matrix, _ = cv2.getOptimalNewCameraMatrix(
                        camera_matrix, dist_coefs, (w, h), alpha=0
                    )
                processed_frame = cv2.undistort(frame, camera_matrix, dist_coefs, None, new_cam_matrix)
            else:
                processed_frame = frame.copy()

            # --- YOLO STRAWBERRY DETECTION ---
            results = model(processed_frame, verbose=False)[0]

            for box in results.boxes:  #Iterate through evry single object boundary detected by the model
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy() #Converts pixel co-ordinates into python numbers
                cls_id = int(box.cls[0])
                label_name = model.names[cls_id]
                confidence = float(box.conf[0]) #Retrieves the probability score assigned to the detection

                if confidence < 0.35:  # Skips bounding box if models certainty is below 40%
                    continue

                #Calculates the centre co-ordinates of the boudnign boxes
                u = (x1 + x2) / 2.0  
                v = (y1 + y2) / 2.0
                w_pixel = x2 - x1

                if w_pixel <= 0:
                    continue

                # Fetch physical width in mm for strawberry classes
                w_real = REAL_WIDTHS_MM.get(label_name, REAL_STRAWBERRY_WIDTH_MM)
                
                # Calculate 3D Spatial Position in mm
                Z = (fx * w_real) / w_pixel
                X = ((u - cx) * Z) / fx
                Y = ((v - cy) * Z) / fy

                # Draw bounding boxes and coordinates
                cv2.rectangle(processed_frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                coord_str = f"{label_name} ({confidence*100:.0f}%): X={int(X)} Y={int(Y)} Z={int(Z)}mm"
                cv2.putText(processed_frame, coord_str, (int(x1), max(20, int(y1) - 10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

            # Display Lens Status HUD
            status = "UNDISTORT: ON" if apply_undistort else "UNDISTORT: OFF (Raw Crop)"
            cv2.putText(processed_frame, status, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)

            cv2.imshow("ESP32-S3 Vision Pipeline", processed_frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('u'):
                apply_undistort = not apply_undistort
            elif key == 27:  # ESC key
                break

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()