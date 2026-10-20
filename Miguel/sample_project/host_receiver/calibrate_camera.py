import cv2
import numpy as np
import pygetwindow as gw
import mss
import time

# --- CONFIGURATION ---
# Interior corners (internal intersection points, NOT square counts)
CHECKERBOARD = (6, 8)     # Update to match your physical pattern's interior corners
SQUARE_SIZE_MM = 25.0     # 25mm per square side
WINDOW_TITLE = "ESP32-S3 Camera Stream over BLE"

def main():
    print("=== OpenCV BLE Camera Calibration ===")
    print("Searching for window with title containing:", WINDOW_TITLE)
    
    with mss.mss() as sct:
        windows = gw.getWindowsWithTitle(WINDOW_TITLE)
        if not windows:
            print(f"[-] Could not find window with title '{WINDOW_TITLE}'. Make sure Chrome/Edge is open.")
            return

        win = windows[0]
        if win.isMinimized:
            win.restore()

        try:
            win.activate()
        except Exception:
            pass

        print(f"[+] Found window: {win.title}")

        objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
        objp = objp * SQUARE_SIZE_MM

        objpoints = []  #Represents the 3D real world co-ordinates which are stored in an array
        imgpoints = []  #Represents the 2D pixel co-ordinates which are stored in an array
        captured_count = 0

        while True:
            # Captures the full browser window of the camera stream
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

            # Below just crops out the browser tabs, search bar and background margins so we can
            #Isolate the live stream player and capture the images without any obstruction 
            y_start = int(h_full * 0.35)
            y_end = int(h_full * 0.80)
            x_start = int(w_full * 0.30)
            x_end = int(w_full * 0.70)

            frame = full_frame[y_start:y_end, x_start:x_end]
            
            if frame.size == 0:
                frame = full_frame

            #Image ENhancment and corner detection
            # Enhance low-contrast image allowing the camera to find the corners of the checker board
            # even with uneven ligthing conditions 
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
            gray_enhanced = clahe.apply(gray)

            display_frame = frame.copy()

            #Determines if the 6x8 grid image is detected within the camera frame 
            ret, corners = cv2.findChessboardCorners(
                gray_enhanced, 
                CHECKERBOARD, 
                cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE
            )

            # Draw indicators on grid for image capture 
            if ret:
                cv2.drawChessboardCorners(display_frame, CHECKERBOARD, corners, ret)
                status_color = (0, 255, 0)
                status_text = "CHECKERBOARD DETECTED! Press SPACE to capture"
            else:
                status_color = (0, 0, 255)
                status_text = "Searching for board... (Check lighting & focus)"

            cv2.putText(display_frame, f"Captured: {captured_count}/15", (20, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
            cv2.putText(display_frame, status_text, (20, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, status_color, 2)
            cv2.putText(display_frame, "SPACE: Save Frame | ESC: Finish", (20, 90),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

            cv2.imshow("OpenCV Calibration Capture Feed", display_frame)
            key = cv2.waitKey(30) & 0xFF

            if key == 32:  # Spacebar to capute images
                if ret:
                    #refines to sub_pixel decimal accuracy from integer positions
                    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                    corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                    objpoints.append(objp) #Pairs the physical 3d co-ordinates with the captured image coordinates
                    imgpoints.append(corners2)
                    captured_count += 1
                    print(f"[+] Frame {captured_count} captured successfully!")
                else:
                    print("[-] Cannot capture: Checkerboard not detected in frame.")

            elif key == 27 or key == ord('q'):
                break

        cv2.destroyAllWindows()

        if captured_count < 10:
            print(f"[-] Need at least 10 valid captures (got {captured_count}). Calibration aborted.")
            return

        print("\n[+] Calculating intrinsic camera matrix...")
        h, w = gray.shape[:2]
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, (w, h), None, None)
        # calibrate camera:solves the central camera matrix by minimizing the error between actual 3D
        #Ground truth and the 2d captured pixels
        #mtx: the 3 x 3 camera matrix contains the focal length (fx and fy) and optical centre (cx and cy)
        if ret:
            #Compresses the focal length and and optical center so we can use it for the main tracker 
            f_x, f_y = mtx[0, 0], mtx[1, 1]
            c_x, c_y = mtx[0, 2], mtx[1, 2]

        

            print("\n==========================================")
            print("       CALIBRATION SUCCESSFUL!            ")
            print("==========================================")
            print(f"Focal Length (fx, fy)   : ({f_x:.2f}, {f_y:.2f})")
            print(f"Principal Point (cx, cy): ({c_x:.2f}, {c_y:.2f})")
            print("Distortion Coefficients:", dist.ravel())

            np.savez("camera_calib.npz", camera_matrix=mtx, dist_coefs=dist, fx=f_x, fy=f_y, cx=c_x, cy=c_y)
            print("\nSaved parameters to 'camera_calib.npz'")

if __name__ == "__main__":
    main()