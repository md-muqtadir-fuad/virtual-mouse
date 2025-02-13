import cv2
import mediapipe as mp
import pyautogui
import time

cam  = cv2.VideoCapture(0)
face_mesh = mp.solutions.face_mesh.FaceMesh(refine_landmarks = True)
screen_w, screen_h = pyautogui.size()

# Variables for blink detection
prev_x, prev_y = 0, 0
smoothing_factor = 0.2
blink_threshold = 0.008  # Adjust based on your testing
click_cooldown = 1  # Minimum time between clicks in seconds
last_click_time = 0
blink_state = False  # False = Open, True = Closed

while True:
    _ , frame = cam.read()
    frame = cv2.flip(frame,1)
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    output = face_mesh.process(rgb_frame)
    landmark_points = output.multi_face_landmarks
    #print(landmark_points)
    frame_h, frame_w, _ = frame.shape
    
    if landmark_points:
        landmarks = landmark_points[0].landmark
        for id, landmark in enumerate(landmarks[474:478]):
            #print(len(landmarks))
            x = int(landmark.x * frame_w)
            y = int(landmark.y * frame_h)
            cv2.circle(frame,(x,y),3,(0,255,0),-1)
            
            if id == 1:
                target_x = screen_w * landmark.x
                target_y = screen_h * landmark.y
                screen_x = prev_x = (target_x - prev_x) * smoothing_factor
                screen_y = prev_y = (target_y - prev_y) * smoothing_factor
                prev_x , prev_y = screen_x, screen_y 
                pyautogui.moveTo(x,y)
                
        left = [landmarks[145],landmarks[159]]
        for landmark in left:
            x = int(landmark.x * frame_w)
            y = int(landmark.y * frame_h)
            cv2.circle(frame,(x,y),3,(0,0,255),-1)
            #print(x,y)
        print(left[0].y-left[1].y)
        if abs(left[0].y-left[1].y) < 0.01:
            cv2.putText(frame,'Click',(50,50),cv2.FONT_HERSHEY_COMPLEX,1,(0,0,255))
            print('Click')
            current_time = time.time()
            if current_time - last_click_time > 1:
                pyautogui.click()
                #pyautogui.sleep(1)
                last_click_time = current_time
    cv2.imshow("Virtual Eye Controlled Mouse",frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cam.release()
cv2.destroyAllWindows()
