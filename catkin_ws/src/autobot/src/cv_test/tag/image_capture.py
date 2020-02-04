import cv2

cap = cv2.VideoCapture(3)

count=0
while True:
	_, frame = cap.read()
	
	cv2.imshow('frame', frame)
	k = cv2.waitKey(100)

	if k == 27:
		break
	elif k==119: # for 'w': writing
		cv2.imwrite("img"+str(count)+".jpg", frame)
		count+=1
	else:
		continue
		

cv2.destroyAllWindows()
cap.release()

