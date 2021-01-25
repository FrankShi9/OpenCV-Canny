
import cv2
import numpy as np

np.set_printoptions(threshold=np.inf)

cap = cv2.VideoCapture(0)
 
printCnt = True

def main():

    while True:
        _,frame = cap.read()
            # if frame.isEmpty():
            #     break
        frame=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        frame=cv2.blur(frame, (7,7))
        frame=cv2.Canny(frame,0,30,3)
        cv2.imshow("real time edge detect:",frame)
        if printCnt == True:
            print(frame)
            np.savetxt("data.txt", frame)
            printCnt = False
    
        a=cv2.waitKey(30)
        if a == 27:#exit
            break

    cap.release()
    del cv2
    del np



if __name__ == "__main__":
    main()