import cv2

CAMERA_INDEX = 0

# cap is a VideoCapture object we can use to get frames from webcam
cap = cv2.VideoCapture(CAMERA_INDEX)

while True:
  # Capture a frame from the webcam
  _, frame = cap.read()

  # Display that frame (resized to be smaller for convenience)
  cv2.imshow('frame', frame)

  # Quit if user presses q
  if cv2.waitKey(1) & 0xFF == ord('q'):
      break
